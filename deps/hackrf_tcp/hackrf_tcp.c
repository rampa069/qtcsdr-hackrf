/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#else
#include <WinSock2.h>
#include <getopt.h>
#endif

#include <pthread.h>

#include <libhackrf/hackrf.h>

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#define sleep(x) Sleep(x)

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif

static SOCKET s;
int gains[6] = { 0, 8, 16, 24, 32, 40 };
static pthread_t tcp_worker_thread;
static pthread_t command_thread;
static pthread_cond_t exit_cond;
static pthread_mutex_t exit_cond_lock;
static volatile int dead[2] = {0, 0};

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

struct llist {
	char *data;
	size_t len;
	struct llist *next;
};

typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} dongle_info_t;

static hackrf_device *dev = NULL;

int global_numq = 0;
static struct llist *ll_buffers = 0;
int llbuf_num=500;
static int do_exit = 0;

void usage(void)
{
	printf("hackrf_tcp, an I/Q spectrum server for HackRF One transceivers\n\n"
		"Usage:\t[-a listen address]\n"
		"\t[-p listen port (default: 1234)]\n"
		"\t[-f frequency to tune to [Hz]]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-s samplerate in Hz (default: 2048000 Hz)]\n"
		"\t[-d device index (default: 0)]\n");
	exit(1);
}

#ifdef _WIN32
int gettimeofday(struct timeval *tv, void* ignored)
{
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (NULL != tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime;
		tmp <<= 32;
		tmp |= ft.dwLowDateTime;
		tmp /= 10;
		tmp -= 11644473600000000Ui64;
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}

BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		//rtlsdr_cancel_async(dev);
		hackrf_stop_rx(dev);
		hackrf_close(dev);
		sleep(1.2);
		hackrf_init();
		hackrf_open(&dev);
		do_exit = 1;
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	if (!do_exit) {
		//rtlsdr_cancel_async(dev);
		hackrf_stop_rx(dev);
		hackrf_close(dev);
		sleep(1.2);
		hackrf_init();
		hackrf_open(&dev);
		do_exit = 1;
	}
}
#endif

int hackrf_callback(hackrf_transfer *transfer)
{
	//unsigned char *buf, uint32_t len, void *ctx)
	if(!do_exit) {
		struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
		rpt->data = (char*)malloc(transfer->buffer_length);
		memcpy(rpt->data, transfer->buffer, transfer->buffer_length);
		rpt->len = transfer->buffer_length;
		rpt->next = NULL;

		pthread_mutex_lock(&ll_mutex);

		if (ll_buffers == NULL) {
			ll_buffers = rpt;
		} else {
			struct llist *cur = ll_buffers;
			int num_queued = 0;

			while (cur->next != NULL) {
				cur = cur->next;
				num_queued++;
			}

			if(llbuf_num && llbuf_num == num_queued-2){
				struct llist *curelem;

				free(ll_buffers->data);
				curelem = ll_buffers->next;
				free(ll_buffers);
				ll_buffers = curelem;
			}

			cur->next = rpt;

			if (num_queued > global_numq)
				printf("ll+, now %d\n", num_queued);
			else if (num_queued < global_numq)
				printf("ll-, now %d\n", num_queued);

			global_numq = num_queued;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&ll_mutex);
	}
	return 0;
}

static void *tcp_worker(void *arg)
{
	struct llist *curelem,*prev;
	long bytesleft, bytessent, index;
	struct timeval tv= {1,0};
	struct timespec ts;
	struct timeval tp;
	fd_set writefds;
	int r = 0;

	while(1) {
		if(do_exit)
			pthread_exit(0);

		pthread_mutex_lock(&ll_mutex);
		gettimeofday(&tp, NULL);
		ts.tv_sec  = tp.tv_sec+5;
		ts.tv_nsec = tp.tv_usec * 1000;
		r = pthread_cond_timedwait(&cond, &ll_mutex, &ts);
		if(r == ETIMEDOUT) {
			pthread_mutex_unlock(&ll_mutex);
			printf("worker cond timeout\n");
			sighandler(0);
			dead[0]=1;
			pthread_exit(NULL);
		}

		curelem = ll_buffers;
		ll_buffers = 0;
		pthread_mutex_unlock(&ll_mutex);

		while(curelem != 0) {
			bytesleft = curelem->len;
			index = 0;
			bytessent = 0;
			while(bytesleft > 0) {
				FD_ZERO(&writefds);
				FD_SET(s, &writefds);
				tv.tv_sec = 1;
				tv.tv_usec = 0;
				r = select(s+1, NULL, &writefds, NULL, &tv);
				if(r) {
					bytessent = send(s,  &curelem->data[index], bytesleft, 0);
					if (bytessent == SOCKET_ERROR) {
			                        perror("worker socket error");
						sighandler(0);
						dead[0]=1;
						pthread_exit(NULL);
					} else if (do_exit) {
						printf("do_exit\n");
						dead[0]=1;
						pthread_exit(NULL);
					} else {
						bytesleft -= bytessent;
						index += bytessent;
					}
				} else if(do_exit) {
						printf("worker socket bye\n");
						sighandler(0);
						dead[0]=1;
						pthread_exit(NULL);
				}
			}
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}
	}
}

static int set_tuner_amp(hackrf_device *_dev, unsigned int p)
{
	int res = 0;
	res = hackrf_set_amp_enable(_dev, p);
        return res;
}

static int set_tuner_gain(hackrf_device *_dev, int g, unsigned int p)
{
	int res = 0;
	if (g >= 1) {
		res = hackrf_set_lna_gain(_dev, p);
	} else {
		res = hackrf_set_vga_gain(_dev, p);
	}
        return res;
}

static int set_tuner_if(hackrf_device *_dev, unsigned int p)
{
	int res = 0;
	//res = hackrf_set_if_freq(_dev, p);
        return res;
}



static int set_gain_by_index(hackrf_device *_dev, unsigned int index)
{
        int res = 0;
        int count = 6;

        if (count > 0 && (unsigned int)count > index) {
                res = hackrf_set_vga_gain(_dev, gains[index]);
                res = hackrf_set_lna_gain(_dev, gains[index]);
        }

        return res;
}

#ifdef _WIN32
#define __attribute__(x)
#pragma pack(push, 1)
#endif
struct command{
	unsigned char cmd;
	unsigned int param;
}__attribute__((packed));
#ifdef _WIN32
#pragma pack(pop)
#endif
static void *command_worker(void *arg)
{
	int left, received, gain;
	fd_set readfds;
	struct command cmd={0, 0};
	struct timeval tv= {1, 0};
	int r = 0;
	uint32_t tmp;

	while(1) {
		left=sizeof(cmd);
		while(left >0) {
			FD_ZERO(&readfds);
			FD_SET(s, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(s+1, &readfds, NULL, NULL, &tv);
			if(r) {
				received = recv(s, (char*)&cmd+(sizeof(cmd)-left), left, 0);
				if(received == SOCKET_ERROR){
					perror("comm recv socket error");
					sighandler(0);
					dead[1]=1;
					pthread_exit(NULL);
				} else if(do_exit){
					printf("do exit\n");
					dead[1]=1;
					pthread_exit(NULL);
				} else {
					left -= received;
				}
			} else if(do_exit) {
				printf("comm recv bye\n");
				sighandler(0);
				dead[1] = 1;
				pthread_exit(NULL);
			}
		}
		switch(cmd.cmd) {
		case 0x01:
			printf("set freq %ld\n", (long)ntohl(cmd.param));
			hackrf_set_freq(dev, (long)ntohl(cmd.param));
			break;
		case 0x02:
			printf("set sample rate %d\n", ntohl(cmd.param));
			hackrf_set_sample_rate(dev, ntohl(cmd.param));
			break;
		case 0x03:
			printf("set amp mode %d\n", ntohl(cmd.param));
			hackrf_set_amp_enable(dev, ntohl(cmd.param));
			break;
		case 0x04:
			gain = (ntohl(cmd.param) / 10);
			if (gain > 42) {
				gain = 40;
			}
			if (gain < 1 || gain > 100) {
				gain = 0;
			}
			printf("set lna gain %d\n", gain);
			hackrf_set_lna_gain(dev, gain);
			break;
		case 0x05:
			tmp = ntohl(cmd.param);
			if (tmp == 99) {
				printf("set freq correction (%d) -> set tuner amp off \n", ntohl(cmd.param));
				set_tuner_amp(dev, 0);
			}
			else if (tmp == 100) {
				printf("set freq correction (%d) -> set tuner amp on \n", ntohl(cmd.param));
				set_tuner_amp(dev, 1);
			}
			else {
				if (tmp > 62) {
						tmp = 62;
				}
				if (tmp < 0) {
						tmp = 0;
				}
				printf("set freq correction -> set vga gain %d\n", tmp);
				set_tuner_gain(dev, 0, tmp);
			}
			//rtlsdr_set_freq_correction(dev, ntohl(cmd.param));
			break;
		case 0x06:
			tmp = ntohl(cmd.param);
			printf("[ignored] set if stage %d gain %d\n", tmp >> 16, (short)(tmp & 0xffff));
			//rtlsdr_set_tuner_if_gain(dev, tmp >> 16, (short)(tmp & 0xffff));
			break;
		case 0x07:
			printf("[ignored] set test mode %d\n", ntohl(cmd.param));
			//rtlsdr_set_testmode(dev, ntohl(cmd.param));
			break;
		case 0x08:
			printf("[ignored] set agc mode %d\n", ntohl(cmd.param));
			//rtlsdr_set_agc_mode(dev, ntohl(cmd.param));
			break;
		case 0x09:
			printf("[ignored] set direct sampling %d\n", ntohl(cmd.param));
			//rtlsdr_set_direct_sampling(dev, ntohl(cmd.param));
			break;
		case 0x0a:
			printf("[ignored] set offset tuning %d\n", ntohl(cmd.param));
			//rtlsdr_set_offset_tuning(dev, ntohl(cmd.param));
			break;
		case 0x0b:
			printf("[ignored] set rtl xtal %d\n", ntohl(cmd.param));
			//rtlsdr_set_xtal_freq(dev, ntohl(cmd.param), 0);
			break;
		case 0x0c:
			printf("[ignored] set tuner xtal %d\n", ntohl(cmd.param));
			//rtlsdr_set_xtal_freq(dev, 0, ntohl(cmd.param));
			break;
		case 0x0d:
			printf("set tuner gain by index %d\n", ntohl(cmd.param));
			set_gain_by_index(dev, ntohl(cmd.param));
			break;
		/* HackRF TCP Exclusive */
		case 0xb0:
			printf("set tuner amp %d\n", ntohl(cmd.param));
			set_tuner_amp(dev, ntohl(cmd.param));
			break;
		case 0xb1:
			printf("set tuner vga gain %d\n", ntohl(cmd.param));
			set_tuner_gain(dev, 0, ntohl(cmd.param));
			break;
		case 0xb2:
			printf("set tuner lna gain %d\n", ntohl(cmd.param));
			set_tuner_gain(dev, 1, ntohl(cmd.param));
			break;
		case 0xb3:
			printf("set intermediate freq %d\n", ntohl(cmd.param));
			set_tuner_if(dev, ntohl(cmd.param));
			break;
		default:
			break;
		}
		cmd.cmd = 0xff;
	}
}
int main(int argc, char **argv)
{
	int r, opt;
	char* addr = "127.0.0.1";
	int port = 1234;
	long frequency = 100000000;
	int samp_rate = 2048000;
	struct sockaddr_in local, remote;
	uint32_t dev_index = 0;
	int gain = 0;
	struct llist *curelem,*prev;
	uint8_t board_id = BOARD_ID_INVALID;
	char version[255 + 1];
	pthread_attr_t attr;
	void *status;
	struct timeval tv = {1,0};
	struct linger ling = {1,0};
	SOCKET listensocket;
	socklen_t rlen;
	fd_set readfds;
	dongle_info_t dongle_info;
#ifdef _WIN32
	WSADATA wsd;
	u_long blockmode = 1;
	int i = WSAStartup(MAKEWORD(2,2), &wsd);
#else
	struct sigaction sigact, sigign;
#endif

	while ((opt = getopt(argc, argv, "a:p:f:g:s:n:d:x")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = atoi(optarg);
			break;
		case 'f':
			frequency = (long)atof(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atof(optarg);
			break;
		case 'a':
			addr = optarg;
			break;
		case 'p':
			port = atoi(optarg);
			break;
		default:
			usage();
			break;
		}
	}

	if (argc < optind)
		usage();

	//device_count = rtlsdr_get_device_count();

//	if (!device_count) {
//		fprintf(stderr, "No supported devices found.\n");
//		exit(1);
//	}

//	printf("Found %d device(s).\n", device_count);
	hackrf_init();
	hackrf_open(&dev);
	if (NULL == dev) {
	fprintf(stderr, "Failed to open hackrf device #%d.\n", dev_index);
		exit(1);
	}
	r = hackrf_board_id_read(dev,&board_id);
        if (r != HACKRF_SUCCESS) {
                fprintf(stderr, "hackrf_board_id_read() failed: %s (%d)\n",
                                hackrf_error_name(r), r);
                exit(1);
        }

        r = hackrf_version_string_read(dev, &version[0], 255);
        if (r != HACKRF_SUCCESS) {
                fprintf(stderr, "hackrf_version_string_read() failed: %s (%d)\n",
                                hackrf_error_name(r), r);
                exit(1);
        }

	printf("Using HackRF %s with firmware %s\n", hackrf_board_id_name(board_id), version);
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigign.sa_handler = SIG_IGN;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigign, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif
	/* Set the sample rate */
	r = hackrf_set_sample_rate(dev, samp_rate);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");

	/* Set the frequency */
	r = hackrf_set_freq(dev, frequency);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
	else
		fprintf(stderr, "Tuned to %ld Hz.\n", frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		hackrf_set_lna_gain(dev,24);
		hackrf_set_vga_gain(dev,24);
		hackrf_set_amp_enable(dev,1);
	} else {
		/* Enable manual gain */
		hackrf_set_amp_enable(dev,1);
		hackrf_set_lna_gain(dev,gain);
		hackrf_set_vga_gain(dev,gain);
	}

	/* Reset endpoint before we start reading from it (mandatory) */
//	r = rtlsdr_reset_buffer(dev);

	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_mutex_init(&ll_mutex, NULL);
	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_cond_init(&cond, NULL);
	pthread_cond_init(&exit_cond, NULL);

	memset(&local,0,sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.s_addr = inet_addr(addr);

	listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	r = 1;
	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	bind(listensocket,(struct sockaddr *)&local,sizeof(local));

	#ifdef _WIN32
	ioctlsocket(listensocket, FIONBIO, &blockmode);
	#else
	r = fcntl(listensocket, F_GETFL, 0);
	r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
	#endif

	while(1) {
		printf("listening...\n");
		printf("Use the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
		       "(gr-osmosdr) source\n"
		       "to receive samples in GRC and control "
		       "rtl_tcp parameters (frequency, gain, ...).\n",
		       addr, port);
		listen(listensocket,1);

		while(1) {
			FD_ZERO(&readfds);
			FD_SET(listensocket, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(listensocket+1, &readfds, NULL, NULL, &tv);
			if(do_exit) {
				goto out;
			} else if(r) {
				rlen = sizeof(remote);
				s = accept(listensocket,(struct sockaddr *)&remote, &rlen);
				break;
			}
		}

		setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

		printf("client accepted!\n");

		memset(&dongle_info, 0, sizeof(dongle_info));
		memcpy(&dongle_info.magic, "RTL0", 4);
		dongle_info.tuner_type = htonl(1);
		dongle_info.tuner_gain_count = htonl(5);

		r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
		if (sizeof(dongle_info) != r)
			printf("failed to send dongle information\n");

		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
		r = pthread_create(&command_thread, &attr, command_worker, NULL);
		pthread_attr_destroy(&attr);

//		r = rtlsdr_read_async(dev, rtlsdr_callback, NULL, buf_num, 0);
		hackrf_start_rx(dev, hackrf_callback, NULL);

		if(!dead[0])
			pthread_join(tcp_worker_thread, &status);
		dead[0]=0;

		if(!dead[1])
			pthread_join(command_thread, &status);
		dead[1]=0;

		closesocket(s);

		printf("all threads dead..\n");
		curelem = ll_buffers;
		ll_buffers = 0;

		while(curelem != 0) {
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}

		do_exit = 0;
		global_numq = 0;
	}

out:
	hackrf_close(dev);
	closesocket(listensocket);
	closesocket(s);
	#ifdef _WIN32
	WSACleanup();
	#endif
	printf("bye!\n");
	return r >= 0 ? r : -r;
}

/*
 * Copyright (c) 2023 Manuel Bouyer
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <err.h>
#include <string.h>
#include <poll.h>
#include <time.h>

#include <host_nmea2000.h>
#include <nmea2000_pgn.h>

int nmea2000_socket;
const struct nmea2000_user_param n2k_param = {
	.n2kuser_address = 128,
	.n2kuser_id = 0x18748,
	.n2kuser_manuf = 0x7fe,
	.n2kuser_device_instance = 0,
	.n2kuser_device_function = 130,
	.n2kuser_device_class = 10,
	.n2kuser_industry_group = 4,
	.n2kuser_system_instance = 0,
};

uint32_t watch_userid = 0;
uint32_t watch_address = 0;
int printf_msg_count = -1;
int is_newline = 1;

struct timeval start_tv;

typedef enum {
	TST_NONE,
	TST_ABS,
	TST_REL,
} tst_type_t;
static tst_type_t tst_type = TST_NONE;

static void
usage()
{
        fprintf(stderr, "usage: n2k_printf [-r|-t] <interface> <devid>\n");
	exit(1); 
}

static void
print_timestamp()
{
	char buf[40];
	struct timeval tv;

	gettimeofday(&tv, NULL);
	switch(tst_type) {
	case TST_NONE:
		return;
	case TST_ABS:
		strftime(buf, sizeof(buf), "%F %H:%M:%S",
		    localtime(&tv.tv_sec));
		printf("%s.%06d ", buf, tv.tv_usec);
		return;
	case TST_REL:
		timersub(&tv, &start_tv, &tv);
		printf("%6d.%06d ", tv.tv_sec, tv.tv_usec);
		return;
	}
}

void
user_receive(struct nmea2000_msg *msg)
{
	int rid;
	struct private_remote_control *ctrl;
	if (msg->n2k_id.iso_pg < 240) {
		switch(msg->n2k_id.iso_pg) {
		case (ISO_ADDRESS_CLAIM >> 8):
			rid = msg->n2k_data[0];
			rid |= msg->n2k_data[1] << 8;
			rid |= (msg->n2k_data[2] & 0x1f) << 16;
			if (rid == watch_userid) {
				if (!is_newline)
					printf("\n");
				print_timestamp();
				printf("found device 0x%x at address %d\n",
				    rid, msg->n2k_id.saddr);
				watch_address = msg->n2k_id.saddr;
				printf_msg_count = 0;
				return;
			}
			break;
		case (PRIVATE_REMOTE_CONTROL >> 8):
			ctrl = (void *)&msg->n2k_data[0];
			if (ctrl->control_type == CONTROL_PRINTF &&
			    ctrl->control_subtype == CONTROL_PRINTF_ADV &&
			    msg->n2k_dlc == CONTROL_PRINTF_ADV_SIZE) {
				rid = ctrl->control_data[0];
				rid |= ctrl->control_data[1] << 8;
				rid |= (ctrl->control_data[2] & 0x1f) << 16;
				if (rid == watch_userid) {
					if (!is_newline)
						printf("\n");
					print_timestamp();
					printf("adv device 0x%x at address %d\n",
					    rid, msg->n2k_id.saddr);
					watch_address = msg->n2k_id.saddr;
					printf_msg_count = 0;
					return;
				}
			}
			break;
		case (PRIVATE_PRINTF_DATA >> 8):
			if (msg->n2k_id.saddr != watch_address)
				return;
				
			for (int i = 0; i < msg->n2k_dlc; i++) {
				if (is_newline)
					print_timestamp();
				is_newline = 0;
				putchar(msg->n2k_data[i]);
				if (msg->n2k_data[i] == '\n')
					is_newline = 1;
			}
			if (printf_msg_count > 0)
				printf_msg_count--;
			break;
		default:
			break;
		}
	}
	return;
}

void
user_handle_iso_request(uint32_t pgn, struct nmea2000_msg *msg)
{
	return;
}

static void
send_iso_request()
{
	struct nmea2000_msg msg;
	uint8_t data[3];

	msg.n2k_id.id = 0;
	msg.n2k_id.priority = NMEA2000_PRIORITY_CONTROL;
	msg.n2k_id.iso_pg = (ISO_REQUEST >> 8);
	msg.n2k_id.saddr = nmea2000_addr;
	msg.n2k_id.daddr = NMEA2000_ADDR_NULL;
	data[0] = (ISO_ADDRESS_CLAIM >> 16);
	data[1] = (ISO_ADDRESS_CLAIM >>  8);
	data[2] = (ISO_ADDRESS_CLAIM & 0xff);
	msg.n2k_data = &data[0];       
	msg.n2k_dlc = 3;
	if (!nmea2000_send_single_frame(&msg)) {
		err(1, "send ISO_REQUEST");
	}
}

static void
send_printf_request()
{
	struct nmea2000_msg msg;
	struct private_remote_control ctrl;

	if (watch_address == 0) {
		printf_msg_count = -1;
		return;
	}

	msg.n2k_id.id = 0;
	msg.n2k_id.priority = NMEA2000_PRIORITY_CONTROL;
	msg.n2k_id.iso_pg = (PRIVATE_REMOTE_CONTROL >> 8);
	msg.n2k_id.saddr = nmea2000_addr;
	msg.n2k_id.daddr = watch_address;
	ctrl.control_type = CONTROL_PRINTF;
	ctrl.control_subtype = CONTROL_PRINTF_ON;
	msg.n2k_data = (void *)&ctrl;       
	msg.n2k_dlc = CONTROL_PRINTF_ON_SIZE;
	if (!nmea2000_send_single_frame(&msg)) {
		warn("send CONTROL_PRINTF_ON");
	}
	printf_msg_count = 50;
}

int
main(int argc, char * const argv[])
{
	int ch;
	uint8_t prev_addr;

	while ((ch = getopt(argc, argv, "rt")) != -1) {
		switch(ch) {
		case 'r':
			tst_type = TST_REL;
			break;
		case 't':
			tst_type = TST_ABS;
			break;
		}
	}
	argc -= optind;
	argv += optind;

	if (argc != 2) {
		usage();
	}

	watch_userid = strtol(argv[1], NULL, 0);

	if ((nmea2000_socket = nmea2000_init(&n2k_param, argv[0])) < 0)
		err(1, "nmea2000_init");

	nmea2000_poll(); /* start address claim */

	struct pollfd fds;
	int ret;

	fds.fd = nmea2000_socket;
	fds.events = POLLRDNORM | POLLRDBAND;
	fds.revents = 0;

	while ((ret = poll(&fds, 1, 500)) >= 0) {
		nmea2000_poll();
		fds.revents = 0;
		if (nmea2000_status == NMEA2000_S_OK)
			break;
	}
	prev_addr = nmea2000_addr;
	gettimeofday(&start_tv, NULL);
	if (tst_type == TST_REL) {
		char buf[40];
		strftime(buf, sizeof(buf), "%F %H:%M:%S",
		    localtime(&start_tv.tv_sec));
		printf("%s.%06d start\n", buf, start_tv.tv_usec);
	}

	/* request the device address */
	send_iso_request();

	/* and start handing incoming packets */
	while ((ret = poll(&fds, 1, 500)) >= 0) {
		nmea2000_poll();
		if (prev_addr != nmea2000_addr) {
			prev_addr = nmea2000_addr;
			/* redirect printf packets to new address */
			printf_msg_count = 0;
		}
		if (printf_msg_count == 0)
			send_printf_request();
	}
	exit(0);
}

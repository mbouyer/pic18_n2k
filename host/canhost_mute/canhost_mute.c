/* $Id: canhost_mute.c,v 1.3 2019/03/10 14:16:53 bouyer Exp $ */
/*
 * Copyright (c) 2017 Manuel Bouyer
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

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <net/if.h>
#ifdef __NetBSD__
#include <netcan/can.h>
#else
#include <linux/can.h>
#include <linux/can/raw.h>
#endif
#include <nmea2000_pgn.h>

int s;

static void
usage()
{
	fprintf(stderr, "usage: canhost_mute <interface> <addr> [0|1]\n");
	exit(1);
}

static void
send_mute(int addr, bool mute)
{
	struct can_frame cf;
	int i, j, r;
	uint16_t csum;

	cf.can_id =  (NMEA2000_PRIORITY_CONTROL << 26) | ((PRIVATE_REMOTE_CONTROL >> 8) << 16) | (addr << 8) | NMEA2000_ADDR_NULL | CAN_EFF_FLAG;
	cf.data[0] = CONTROL_MUTE;
	cf.data[1] = mute;
	cf.can_dlc = CONTROL_MUTE_SIZE;
	if (write(s, &cf, sizeof(cf)) <= 0) {
		err(1, "write PRIVATE_REMOTE_CONTROL to socket");
	}
}

int
main(int argc, const char *argv[])
{
	struct ifreq ifr;
	struct sockaddr_can sa;
	struct can_frame cf;
	int addr;
	bool mute = 0;

	if (argc != 4) {
		usage();
	}
	if (strcmp(argv[3], "0") == 0) {
		mute = 0;
	} else if (strcmp(argv[3], "1") == 0) {
		mute = 1;
	} else {
		usage();
	}

	addr = strtol(argv[2], NULL, 0);
	if (addr >=NMEA2000_ADDR_MAX)
		errx(1, "bad address %s", argv[2]);

	if ((s = socket(AF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		err(1, "CAN socket");
	}
	strncpy(ifr.ifr_name, argv[1], IFNAMSIZ );
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		err(1, "SIOCGIFINDEX for %s", argv[1]);
	}
	sa.can_family = AF_CAN;
	sa.can_ifindex = ifr.ifr_ifindex;
	if (bind(s, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		err(1, "bind socket");
	}

	send_mute(addr, mute);
	exit(0);
}

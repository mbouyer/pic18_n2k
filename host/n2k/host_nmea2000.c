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


#include "host_nmea2000.h"
#include "../../nmea2000_pgn.h"

nmea2000_status_t nmea2000_status;
uint8_t nmea2000_addr;
static struct iso_address_claim_data address_claim_data;
static struct timeval nmea2000_claim_date;
static const struct nmea2000_user_param *nmea2000_user_param;
static int nmea2000_socket = -1;


static void nmea2000_claimaddr(uint8_t, uint8_t);

uint8_t canbus_mute;

static inline int
nmea2000_do_receive()
{
	uint32_t pgn;
	int i;
	struct nmea2000_frame f;
	struct nmea2000_msg msg;

	i = read(nmea2000_socket, &f, sizeof(f));
	if (i < 0) {
		err(1, "read from socket");
	}
	if (i == 0)
		return 0;

	if ((f.n2kf_id.id & CAN_EFF_FLAG) == 0)
		return 1;

	if (f.n2kf_id.iso_pg < 240) {
		if (f.n2kf_id.daddr != nmea2000_addr &&
		    f.n2kf_id.daddr != NMEA2000_ADDR_GLOBAL) {
			return 1;
		}
	}
	if (f.n2kf_id.page == 0) {
		switch(f.n2kf_id.iso_pg) {
		case (ISO_ADDRESS_CLAIM >> 8):
			if (f.n2kf_dlc != 8)
				break;
			if (f.n2kf_id.saddr != nmea2000_addr)
				break;
			for (i = 7; i >= 0; i--) {
				if (f.n2kf_data[i] < address_claim_data.name[i]) {
					/* we loose */
					nmea2000_addr++;
					if (nmea2000_addr >= NMEA2000_ADDR_MAX)
						nmea2000_addr = 0;
					nmea2000_status = NMEA2000_S_IDLE;
					break;
				} else if (f.n2kf_data[i] >
				    address_claim_data.name[i]) {
					break;
				}
			}
			/* defend our address, or send new claim */
			nmea2000_claimaddr(nmea2000_addr, NMEA2000_ADDR_GLOBAL);
			break;
		case (ISO_REQUEST >> 8):
			pgn = ((uint32_t)f.n2kf_data[2] << 16) |
			    ((uint32_t)f.n2kf_data[1] << 8) | f.n2kf_data[0];
			if (pgn == ISO_ADDRESS_CLAIM) {
				nmea2000_claimaddr(nmea2000_addr, f.n2kf_id.saddr);
			} else {
				msg.n2k_id = f.n2kf_id;
				msg.n2k_dlc = f.n2kf_dlc;
				msg.n2k_data = &f.n2kf_data[0];
				user_handle_iso_request(pgn, &msg);
			}
			return 1;
		case (PRIVATE_REMOTE_CONTROL >> 8):
			if (f.n2kf_data[0] == CONTROL_RESET &&
			    f.n2kf_dlc == CONTROL_RESET_SIZE) {
				fprintf(stderr, "reset, exiting\n");
				exit(1);
				break;
			} else if (f.n2kf_data[0] == CONTROL_MUTE &&
			    f.n2kf_dlc == CONTROL_MUTE_SIZE) {
				canbus_mute = f.n2kf_data[1] & 0x1;
				return 1;
			}
			/* FALLTHROUGH */
		default:
			break;
		}
		msg.n2k_id = f.n2kf_id;
		msg.n2k_dlc = f.n2kf_dlc;
		msg.n2k_data = &f.n2kf_data[0];
		user_receive(&msg);
	} else {
		msg.n2k_id = f.n2kf_id;
		msg.n2k_dlc = f.n2kf_dlc;
		msg.n2k_data = &f.n2kf_data[0];
		user_receive(&msg);
	}
	return 1;
}

static int
nmea2000_send_control(struct nmea2000_msg *msg)
{
	struct nmea2000_frame f;

	msg->n2k_id.saddr = nmea2000_addr;

	memset(&f, 0, sizeof(f));

	f.n2kf_id = msg->n2k_id;
	f.n2kf_id.id |= CAN_EFF_FLAG;
	f.n2kf_dlc = msg->n2k_dlc;

	memcpy(&f.n2kf_data[0], msg->n2k_data, msg->n2k_dlc);
	if (write(nmea2000_socket, &f, sizeof(f)) <= 0) {
		warn("write can_msg to socket");
		return 0;
	}
	return 1;
}

int
nmea2000_send_single_frame(struct nmea2000_msg *msg)
{

	if (nmea2000_status != NMEA2000_S_OK || canbus_mute) 
		return 0;

	return nmea2000_send_control(msg);
}


static void
nmea2000_claimaddr(uint8_t saddr, uint8_t daddr)
{
	struct nmea2000_msg address_claim_msg;

	nmea2000_addr = saddr;

	address_claim_msg.n2k_id.id = 0;
	address_claim_msg.n2k_id.priority = NMEA2000_PRIORITY_CONTROL;
	address_claim_msg.n2k_id.iso_pg = (ISO_ADDRESS_CLAIM >> 8);
	address_claim_msg.n2k_id.daddr = daddr;
	address_claim_data.name[0] =
	    (nmea2000_user_param->n2kuser_id) & 0xff;
	address_claim_data.name[1] =
	    (nmea2000_user_param->n2kuser_id >> 8) & 0xff;
	address_claim_data.name[2] =
	    (nmea2000_user_param->n2kuser_id >> 16) & 0x1f;
	address_claim_data.name[2] |=
	    (nmea2000_user_param->n2kuser_manuf << 5) & 0xe0;
	address_claim_data.name[3] =
	    nmea2000_user_param->n2kuser_manuf >> 3;
	address_claim_data.name[4] =
	    nmea2000_user_param->n2kuser_device_instance;
	address_claim_data.name[5] =
	    nmea2000_user_param->n2kuser_device_function;
	address_claim_data.name[6] =
	    nmea2000_user_param->n2kuser_device_class << 1;
	address_claim_data.name[7] =
	    0x80 | (nmea2000_user_param->n2kuser_industry_group << 4) |
	    nmea2000_user_param->n2kuser_system_instance;
	address_claim_msg.n2k_data = &address_claim_data.name[0];
	address_claim_msg.n2k_dlc = sizeof(address_claim_data);
	while (! nmea2000_send_control(&address_claim_msg))
		;
	if (nmea2000_status == NMEA2000_S_IDLE) {
		nmea2000_status = NMEA2000_S_CLAIMING;
		gettimeofday(&nmea2000_claim_date, NULL);
	}
}

void
nmea2000_poll()
{
	struct timeval tv, tv_res;
	struct pollfd fds;
	int ret;

	fds.fd = nmea2000_socket;
	fds.events = POLLRDNORM | POLLRDBAND;
	fds.revents = 0;

	while ((ret = poll(&fds, 1, 0)) > 0) {
		if (nmea2000_do_receive() == 0)
			break;
		fds.revents = 0;
	}
	if (ret < 0) {
		err(1, "poll on socket");
	}

	gettimeofday(&tv, NULL);
	switch(nmea2000_status) {
	case NMEA2000_S_IDLE:
		if (nmea2000_addr == NMEA2000_ADDR_NULL)
			nmea2000_claimaddr(nmea2000_user_param->n2kuser_address,
			    NMEA2000_ADDR_GLOBAL);
		else 
			nmea2000_claimaddr(nmea2000_addr,
			    NMEA2000_ADDR_GLOBAL);
		break;

	case NMEA2000_S_CLAIMING:
		timersub(&tv, &nmea2000_claim_date, &tv_res);
		if (tv_res.tv_sec > 0 || tv_res.tv_usec > 250000) {
			nmea2000_status = NMEA2000_S_OK;
		}
		break;
	default:
		break;
	}
}

int
nmea2000_init(const struct nmea2000_user_param *p, const char *intf)
{
	nmea2000_user_param = p;
	struct ifreq ifr;
	struct sockaddr_can sa;

	if ((nmea2000_socket = socket(AF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		err(1, "CAN socket"); 
	}
	strncpy(ifr.ifr_name, intf, IFNAMSIZ );
	if (ioctl(nmea2000_socket, SIOCGIFINDEX, &ifr) < 0) {
		err(1, "SIOCGIFINDEX for %s", intf);
	}
	sa.can_family = AF_CAN;       
	sa.can_ifindex = ifr.ifr_ifindex;
	if (bind(nmea2000_socket, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		err(1, "bind socket");
	}

	nmea2000_status = NMEA2000_S_IDLE;
	nmea2000_addr = NMEA2000_ADDR_NULL;
	canbus_mute = 0;
	return nmea2000_socket;
}

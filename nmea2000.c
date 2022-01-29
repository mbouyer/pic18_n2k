/* $Id: nmea2000.c,v 1.4 2017/07/23 11:14:56 bouyer Exp $ */
/*
 * Copyright (c) 2021 Manuel Bouyer
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

#ifdef __XC
#include <xc.h>
#endif

#include "nmea2000.h"
#include "nmea2000_pgn.h"
#include "nmea2000_user.h"
#include <stdio.h>
union nmea2000_id rid;

nmea2000_status_t nmea2000_status;
unsigned char nmea2000_addr;
unsigned char canbus_mute;
static unsigned char nmea2000_claim_date;
static struct iso_address_claim_data address_claim_data;

static void nmea2000_claimaddr(unsigned char, unsigned char);
static inline void nmea2000_do_receive(void);

#ifdef PIC_ECAN
#include "nmea2000_pic18_ecan.c"
#else
#include "nmea2000_pic18_can.c"
#endif

static inline void
nmea2000_do_receive(void)
{
	unsigned long pgn;
	signed char i;
	
	if (rid.iso_pg < 240) {
		if (rid.daddr != nmea2000_addr &&
		    rid.daddr != NMEA2000_ADDR_GLOBAL) {
			return;
		}
	}
	if (rid.page == 0) {
		switch(rid.iso_pg) {
		case (ISO_ADDRESS_CLAIM >> 8):
			if (rdlc != 8)
				break;
			if (rid.saddr != nmea2000_addr)
				break;
			for (i = 7; i >= 0; i--) {
				if (rdata[i] < address_claim_data.name[i]) {
					/* we loose */
					nmea2000_addr++;
					if (nmea2000_addr >= NMEA2000_ADDR_MAX)
						nmea2000_addr = 0;
					nmea2000_status = NMEA2000_S_IDLE;
					break;
				} else if (rdata[i] >
				    address_claim_data.name[i]) {
					break;
				}
			}
			/* defend our address, or send new claim */
			nmea2000_claimaddr(nmea2000_addr, NMEA2000_ADDR_GLOBAL);
			break;
		case (ISO_REQUEST >> 8):
			pgn = ((unsigned long)rdata[2] << 16) |
			    ((unsigned int)rdata[1] << 8) | rdata[0];
			if (pgn == ISO_ADDRESS_CLAIM) {
				nmea2000_claimaddr(nmea2000_addr, rid.saddr);
			} else {
				user_handle_iso_request(pgn);
			}
			break;
		case (PRIVATE_REMOTE_CONTROL >> 8):
			if (rdata[0] == CONTROL_RESET &&
			    rdlc == CONTROL_RESET_SIZE) {
				__asm__("reset");
				break;
			} else if (rdata[0] == CONTROL_MUTE &&
			    rdlc == CONTROL_MUTE_SIZE) {
				canbus_mute = rdata[1] & 0x1;
				break;
			}
			/* FALLTHROUGH */
		default:
			user_receive();
		}
	} else {
		user_receive();
	}
}

static void
nmea2000_claimaddr(unsigned char saddr, unsigned char daddr)
{
	static struct nmea2000_msg address_claim_msg;

	nmea2000_addr = saddr;

	address_claim_msg.id.id = 0;
	address_claim_msg.id.priority = NMEA2000_PRIORITY_CONTROL;
	address_claim_msg.id.iso_pg = (ISO_ADDRESS_CLAIM >> 8);
	address_claim_msg.id.daddr = daddr;
	address_claim_data.name[0] = (NMEA2000_USER_ID) & 0xff;
	address_claim_data.name[1] = (NMEA2000_USER_ID >> 8) & 0xff;
	address_claim_data.name[2] = (NMEA2000_USER_ID >> 16) & 0x1f;
	address_claim_data.name[2] |= (NMEA2000_USER_MANUF << 5) & 0xe0;
	address_claim_data.name[3] = NMEA2000_USER_MANUF >> 3;
	address_claim_data.name[4] = NMEA2000_USER_DEVICE_INSTANCE;
	address_claim_data.name[5] = NMEA2000_USER_DEVICE_FUNCTION;
	address_claim_data.name[6] = NMEA2000_USER_DEVICE_CLASS << 1;
	address_claim_data.name[7] = 0x80 | (NMEA2000_USER_INDUSTRY_GROUP << 4) | NMEA2000_USER_SYSTEM_INSTANCE;
	address_claim_msg.data = &address_claim_data.name[0];
	address_claim_msg.dlc = sizeof(address_claim_data);
	while (! nmea2000_send_control(&address_claim_msg))
		;
	if (nmea2000_status == NMEA2000_S_IDLE) {
		nmea2000_status = NMEA2000_S_CLAIMING;
		nmea2000_claim_date = 0;
	}
}

void
nmea2000_poll(unsigned char time)
{
	pic18can_poll(time);
	switch(nmea2000_status) {
	case NMEA2000_S_IDLE:
		if (nmea2000_addr == NMEA2000_ADDR_NULL)
			nmea2000_claimaddr(NMEA2000_USER_ADDRESS,
			    NMEA2000_ADDR_GLOBAL);
		else 
			nmea2000_claimaddr(nmea2000_addr,
			    NMEA2000_ADDR_GLOBAL);
		break;

	case NMEA2000_S_CLAIMING:
		nmea2000_claim_date += time;
		if (nmea2000_claim_date >= 250) {
			nmea2000_status = NMEA2000_S_OK;
			pic18can_set_filter(nmea2000_addr);
		}
		break;
	default:
		break;
	}
}

void
nmea2000_init()
{
	pic18can_init();
	nmea2000_status = NMEA2000_S_ABORT;
	nmea2000_addr = NMEA2000_ADDR_NULL;
	canbus_mute = 0;
}

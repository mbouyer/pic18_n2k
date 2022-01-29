/* $Id: nmea2000.h,v 1.3 2017/07/21 19:23:54 bouyer Exp $ */
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

#ifdef __18F27Q84
#define PIC_ECAN
#endif

#ifdef __XC
#define __data __ram
#endif

#define NMEA2000_DATA_LENGTH	8
#define NMEA2000_DATA_FASTLENGTH 233

union nmea2000_id {
	unsigned long id;
	struct {
		unsigned char saddr	: 8;
		unsigned char daddr	: 8;
		unsigned char iso_pg	: 8;
		unsigned char page	: 1;
		unsigned char		: 1;
		unsigned char priority	: 3;
		unsigned char		: 3;
	};
};

#define ID2PGN(id) (((long)(id).page << 16) | (long)(id).iso_pg << 8 | (long)(id).daddr)
#define PGN2ID(epgn, id) { (id).daddr = ((epgn) & 0xff); (id).iso_pg = ((epgn >> 8) & 0xff) ;  (id).page = ((epgn >> 16) & 1); }

struct nmea2000_msg {
	union nmea2000_id id;
	unsigned char dlc;
	__data unsigned char *data;
};


struct nmea2000_msg_storage {
	union nmea2000_id id;
	unsigned char dlc;
	unsigned char data[NMEA2000_DATA_LENGTH];
};

extern unsigned char nmea2000_addr;

typedef enum {
	NMEA2000_S_OK = 0,
	NMEA2000_S_ABORT,
	NMEA2000_S_RESET,
	NMEA2000_S_IDLE,
	NMEA2000_S_CLAIMING,
} nmea2000_status_t;
extern nmea2000_status_t nmea2000_status; 

extern unsigned char canbus_mute;

extern union nmea2000_id rid;
#ifndef PIC_ECAN
#define rdata ((unsigned char *)&RXB0D0)
#define rdlc RXB0DLC
#else
extern unsigned char *rdata;
extern unsigned char rdlc;
#endif /* PIC_ECAN */

void nmea2000_init(void);
void nmea2000_poll(unsigned char); /* arg : time in ms */
void nmea2000_receive(void);
char nmea2000_send_single_frame(__data struct nmea2000_msg *);
char nmea2000_send_fast_frame(__data struct nmea2000_msg *, unsigned char);
void nmea2000_intr(void);

/* user-implemented callback */
void user_handle_iso_request(unsigned long);
void user_receive(void);

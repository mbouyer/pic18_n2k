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

#include <sys/socket.h>
#include <netcan/can.h>

#define NMEA2000_DATA_LENGTH	8
#define NMEA2000_DATA_FASTLENGTH 223

union nmea2000_id {
	canid_t id;
	struct {
		uint8_t saddr	: 8;
		uint8_t daddr	: 8;
		uint8_t iso_pg	: 8;
		uint8_t page	: 1;
		uint8_t		: 1;
		uint8_t priority : 3;
		uint8_t		: 3;
	};
};

#define ID2PGN(id) (((uint32_t)(id).page << 16) | (uint32_t)(id).iso_pg << 8 | (uint32_t)(id).daddr)
#define PGN2ID(epgn, id) { (id).daddr = ((epgn) & 0xff); (id).iso_pg = ((epgn >> 8) & 0xff) ;  (id).page = ((epgn >> 16) & 1); }

struct nmea2000_msg {
	union nmea2000_id n2k_id;
	uint8_t n2k_dlc;
	uint8_t *n2k_data;
};

struct nmea2000_frame {
	union nmea2000_id n2kf_id;
	uint8_t n2kf_dlc;
        uint8_t __pad;
	uint8_t __res0;
	uint8_t __res1;
	uint8_t n2kf_data[NMEA2000_DATA_LENGTH];
};

extern uint8_t nmea2000_addr;
extern uint8_t canbus_mute;

typedef enum {
	NMEA2000_S_OK = 0,
	NMEA2000_S_IDLE,
	NMEA2000_S_CLAIMING,
} nmea2000_status_t;

extern nmea2000_status_t nmea2000_status; 

struct nmea2000_user_param {
	uint32_t n2kuser_address; /* device suugected address */
	uint32_t n2kuser_id; /* device ID, 21 bits */
	uint16_t n2kuser_manuf; /* manufacturer code, 11 bits */
	uint8_t  n2kuser_device_instance; /* 8 bits */
	uint8_t  n2kuser_device_function; /* 8 bits */
	uint8_t  n2kuser_device_class; /* 7 bits */
	uint8_t  n2kuser_industry_group; /* 3 bits */
	uint8_t  n2kuser_system_instance; /* 4 bits */
};

int nmea2000_init(const struct nmea2000_user_param *, const char *);
void nmea2000_poll(void);
void nmea2000_receive(struct nmea2000_msg *);
int nmea2000_send_single_frame(struct nmea2000_msg *);
int nmea2000_send_fast_frame(struct nmea2000_msg *, uint8_t);
void nmea2000_intr(void);

/* user-implemented callback */
void user_handle_iso_request(uint32_t, struct nmea2000_msg *);
void user_receive(struct nmea2000_msg *);

#define SIDINC(s) { \
	s++; \
	if (s == 0xfe) \
		s = 0;\
	}

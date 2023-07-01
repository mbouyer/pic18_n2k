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

#define N2KP_BUFSIZE 128
#define N2KP_BUFSIZE_MASK 0x7f
char n2kp_txbuf[N2KP_BUFSIZE];

unsigned char n2kp_txbuf_prod;
volatile unsigned char n2kp_txbuf_cons;
unsigned char n2kp_eol;
unsigned char n2kp_xmit;
unsigned char printf_daddr, printf_count;
static struct nmea2000_msg n2kp_msg;
static unsigned char n2kp_buf[NMEA2000_DATA_LENGTH];

void
nema2000_printf_control()
{
	if (rdata[1] == CONTROL_PRINTF_ON && rdlc == CONTROL_PRINTF_ON_SIZE) {
		printf_daddr = rid.saddr;
		printf_count = 100;
	}
}

void
nema2000_printf_init() {
	n2kp_txbuf_prod = n2kp_txbuf_cons = 0;
	n2kp_xmit = 0;
	n2kp_eol = 0;
	printf_daddr = 0;
}

void
nema2000_printf_adv()
{
	__data struct private_remote_control *d = (void *)&n2kp_buf[0];

	d->control_type = CONTROL_PRINTF;  
	d->control_subtype = CONTROL_PRINTF_ADV;
	d->control_data[0] = (NMEA2000_USER_ID) & 0xff;
	d->control_data[1] = (NMEA2000_USER_ID >> 8) & 0xff;
	d->control_data[2] = (NMEA2000_USER_ID >> 16) & 0x1f;

	n2kp_msg.id.id = 0;
	n2kp_msg.id.iso_pg = (PRIVATE_REMOTE_CONTROL >> 8) & 0xff;
	n2kp_msg.id.daddr = NMEA2000_ADDR_GLOBAL;
	n2kp_msg.id.priority = NMEA2000_PRIORITY_INFO;
	n2kp_msg.dlc = CONTROL_PRINTF_ADV_SIZE;
	n2kp_msg.data = &n2kp_buf[0];
	if (!nmea2000_send_single_frame(&n2kp_msg)) {
		unsigned char _printf_daddr;
		_printf_daddr = printf_daddr;
		printf_daddr = 0;
		printf("send CONTROL_PRINTF_ADV failed\n");
		printf_daddr = _printf_daddr;
	}
}

static char
nmea2000_do_xmix()
{
	unsigned char new_n2kp_txbuf_cons, new_n2kp_eol;
	unsigned char i;

	new_n2kp_eol = n2kp_eol;

	for (i = 0, new_n2kp_txbuf_cons = n2kp_txbuf_cons;
	     i < NMEA2000_DATA_LENGTH && new_n2kp_txbuf_cons != n2kp_txbuf_prod;
	     i++) {
		n2kp_buf[i] = n2kp_txbuf[new_n2kp_txbuf_cons];
		new_n2kp_txbuf_cons =
		    (new_n2kp_txbuf_cons + 1) & N2KP_BUFSIZE_MASK;
		if (n2kp_buf[i] == '\n') {
			if (new_n2kp_eol != 0)
				new_n2kp_eol--;
		}
	}
	n2kp_msg.id.id = 0;
	n2kp_msg.id.iso_pg = (PRIVATE_PRINTF_DATA >> 8) & 0xff;
	n2kp_msg.id.daddr = printf_daddr;
	n2kp_msg.id.priority = NMEA2000_PRIORITY_INFO;
	n2kp_msg.dlc = i;
	n2kp_msg.data = (void *)&n2kp_buf[0];
	if (nmea2000_send_single_frame(&n2kp_msg)) {
		n2kp_txbuf_cons = new_n2kp_txbuf_cons;
		new_n2kp_eol = n2kp_eol;
		printf_count--;
		if (printf_count == 0) {
			printf_daddr = 0;
		}
		return 1;
	} else {
		n2kp_xmit = 1;
		return 0;
	}
}

static void
nmea2000_xmix()
{
	while (1) {
		unsigned char bufsize =
		    (n2kp_txbuf_prod - n2kp_txbuf_cons) & N2KP_BUFSIZE_MASK;
		if (bufsize == 0) {
			n2kp_xmit = 0;
			n2kp_eol = 0;
			return;
		}
		if (bufsize < NMEA2000_DATA_LENGTH && n2kp_eol == 0) {
			/* nothing to send (yet) */
			n2kp_xmit = 0;
			return;
		}
		if (nmea2000_do_xmix() == 0) {
			n2kp_xmit = 1;
			return;
		}
	}
}

void
nmea2000_putchar(char c)
{
	unsigned char new_n2kp_txbuf_prod = (n2kp_txbuf_prod + 1) & N2KP_BUFSIZE_MASK;
	unsigned char bufsize;
	if (printf_daddr == 0 ||
	    canbus_mute ||
	    nmea2000_status != NMEA2000_S_OK)
		return;

	bufsize = (new_n2kp_txbuf_prod - n2kp_txbuf_cons) & N2KP_BUFSIZE_MASK;
	if (bufsize == 0)
		return; /* full, drop char */

	n2kp_txbuf[n2kp_txbuf_prod] = c;
	n2kp_txbuf_prod = new_n2kp_txbuf_prod;
	if (c == '\n')
		n2kp_eol++;

	if (bufsize >= NMEA2000_DATA_LENGTH || n2kp_eol)
		nmea2000_xmix();
}

static inline void
nema2000_printf_poll()
{
	if (n2kp_xmit)
		nmea2000_xmix();
}

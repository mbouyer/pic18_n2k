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

/* CAN backend for pic18 ECAN modules (e.g. pic18f27q84) */

#ifdef _18F27Q84
#define CANRAMBASE 0x3800
#endif

#define CAN_TXSQIZE	8
#define CAN_FIFO1SIZE	8
#define CAN_FIFO2SIZE	8
#ifdef NEMA2000_USE_FAST_FRAME
#define CAN_FIFO3SIZE	30 /* a complete frame */
#else
#define CAN_FIFO3SIZE 1
#endif

#define EFIFO_SIZE (16 * CAN_TXSQIZE + 16 * CAN_FIFO1SIZE + 16 * CAN_FIFO2SIZE + 16 * CAN_FIFO3SIZE)

static unsigned char ecanfifo[EFIFO_SIZE] __at(CANRAMBASE);

static uint16_t nmea2000_xmit_date; 

static inline void
pic18can_config_mode(void)
{
	C1CONTbits.REQOP = 0x04;
	while (C1CONUbits.OPMOD != 0x04)
		;
}

static inline void
pic18can_normal_mode(void)
{
	C1CONTbits.REQOP = 0x6;
	while (C1CONUbits.OPMOD != 0x6)
		;
}

static inline void
pic18can_set_filter(unsigned char addr)
{
	C1FLTCON0T = 0;
	C1FLTOBJ3L = 0; /* sid 7-0 */
	C1FLTOBJ3H = 0; /* sid 8-10, eid 4-0 */
	C1FLTOBJ3U = (addr << 3) & 0xf8; /* eid 5-12 */
	C1FLTOBJ3T = _C1FLTOBJ0T_EXIDE_MASK |
	    ((addr >> 5) & 0x1f); /* also eid 13 - 17 */
	C1FLTCON0T = 0x81;
}

unsigned char *rdata;
unsigned char rdlc;

void
nmea2000_receive()
{
	while (C1INTLbits.RXIF) {
		unsigned char *fifo;
		fifo = (unsigned char *)(C1FIFOUA1L | (C1FIFOUA1H << 8));

		rid.id = (uint32_t)((fifo[1] >> 3) | (fifo[2] << 5) |
		    ((fifo[3] & 0x7) << 13));

		rid.iso_pg = ((fifo[3] >> 3) & 0x3) | ((fifo[0] << 2) & 0xfc);

		rid.page = (fifo[0] & 0x40) ? 1 : 0;
		rid.priority = (fifo[1] & 0x7);

		rdlc = fifo[4] & 0xf;
		rdata = &fifo[8];
		nmea2000_do_receive();

		C1FIFOCON1Hbits.UINC = 1;
	}
	C1INTUbits.RXIE = 1;
}

char
nmea2000_send_single_frame(__data struct nmea2000_msg *msg)
{
	unsigned char i;
	unsigned char *fifo;
	fifo = (unsigned char *)(C1FIFOUA2L | (C1FIFOUA2H << 8));

	if (nmea2000_status != NMEA2000_S_OK || canbus_mute)
		return 0;

	if (C1FIFOSTA2Lbits.TFNRFNIF == 0) {
		return 0;
	}

	fifo[0] = (msg->id.id >> 18) & 0xff;
	fifo[1] = ((msg->id.id >> 26) & 0x7) | ((nmea2000_addr << 3) & 0xf8);
	fifo[2] = ((nmea2000_addr >> 5) & 0x7) | ((msg->id.id >> 5) & 0xf8);
	fifo[3] = (msg->id.id >> 13) & 0x1f;
	fifo[4] = msg->dlc | 0x10;

	for (i = 0; i < msg->dlc; i++)
		fifo[i + 8] = msg->data[i];

	C1FIFOCON2H = 0x3; /* set UNC and TXREQ */
	nmea2000_xmit_date = 0;
	return 1;
}

#ifdef NEMA2000_USE_FAST_FRAME

char
nmea2000_send_fast_frame(__data struct nmea2000_msg *msg, unsigned char id)
{
	unsigned char i, j, n;
	unsigned char len = msg->dlc;
	unsigned char *data = msg->data;
	unsigned char *fifo;

	if (nmea2000_status != NMEA2000_S_OK || canbus_mute)
		return 0;

	for (n  = 0; len > 0; n++) {
		fifo = (unsigned char *)(C1FIFOUA3L | (C1FIFOUA3H << 8));
		if (C1FIFOSTA3Lbits.TFNRFNIF == 0)
			return 0;

		fifo[0] = (msg->id.id >> 18) & 0xff;
		fifo[1] = ((msg->id.id >> 26) & 0x7) |
		    ((nmea2000_addr << 3) & 0xf8);
		fifo[2] = ((nmea2000_addr >> 5) & 0x7) |
		    ((msg->id.id >> 5) & 0xf8);
		fifo[3] = (msg->id.id >> 13) & 0x1f;

		fifo[8] = ((id << 5) & 0xe) | n ;
		if (n == 0) {
			fifo[9] = len;
			j = len;
			if (j > 6)
				j = 6;
			for (i = 0; i < j; i++)
				fifo[i+10] = data[i];
			len -= j;
			data += j;
			fifo[4] = (j + 2) | 0x10;
		} else {
			j = len;
			if (j > 7)
				j = 7;
			for (i = 0; i < j; i++)
				fifo[i+9] = data[i];
			len -= j;
			data += j;
			fifo[4] = (j + 1) | 0x10;
		}
		C1FIFOCON3H = 0x3; /* set UNC and TXREQ */
	}
	nmea2000_xmit_date = 0;
	return 1;
}
#endif /* NEMA2000_USE_FAST_FRAME */

static char
nmea2000_send_control(struct nmea2000_msg *msg)
{
	unsigned char i;
	unsigned char *fifo;
	fifo = (unsigned char *)(C1TXQUAL | (C1TXQUAH << 8));

	if (C1TXQSTALbits.TXQNIF == 0)
		return 0;

	fifo[0] = (msg->id.id >> 18) & 0xff;
	fifo[1] = ((msg->id.id >> 26) & 0x7) | ((nmea2000_addr << 3) & 0xf8);
	fifo[2] = ((nmea2000_addr >> 5) & 0x7) | ((msg->id.id >> 5) & 0xf8);
	fifo[3] = (msg->id.id >> 13) & 0x1f;
	fifo[4] = msg->dlc | 0x10;
	for (i = 0; i < msg->dlc; i++)
		fifo[i + 8] = msg->data[i];

	C1TXQCONH = 0x3; /* set UNC and TXREQ */
	nmea2000_xmit_date = 0;
	return 1;
}

static inline void
pic18can_poll(unsigned char time)
{
	switch(nmea2000_status) {
	case NMEA2000_S_OK:
		/* check transmit fifos for errors. If any, reset */
		if (C1TXQSTALbits.TXERR ||
		    C1FIFOSTA2Lbits.TXERR ||
		    C1FIFOSTA3Lbits.TXERR) {
			goto abort;
		}

		if (C1TXQCONHbits.TXREQ == 0 &&
		    C1FIFOCON2Hbits.TXREQ == 0 &&
		    C1FIFOCON3Hbits.TXREQ == 0) {
			/* no transmit pending */
			return;
		}

		nmea2000_xmit_date += time;
		if (nmea2000_xmit_date < 1000) {
			/* no timeout yet */
			return;
		}
abort:
		C1TXQCONHbits.TXREQ = 0;
		C1FIFOCON2Hbits.TXREQ = 0;
		C1FIFOCON3Hbits.TXREQ = 0;
		nmea2000_status = NMEA2000_S_ABORT;
		pic18can_set_filter(NMEA2000_ADDR_GLOBAL);
		/* FALLTRHOUGH */
	case NMEA2000_S_ABORT:
		if (C1TXQCONHbits.TXREQ ||
		    C1FIFOCON2Hbits.TXREQ ||
		    C1FIFOCON3Hbits.TXREQ) {
			/* still aborting transmits, wait */
			return;
		}
		C1TXQCONHbits.FRESET = 1;
		C1FIFOCON2Hbits.FRESET = 1;
		C1FIFOCON3Hbits.FRESET = 1;
		nmea2000_status = NMEA2000_S_RESET;
		/* FALLTRHOUGH */
	case NMEA2000_S_RESET:
		if (C1TXQCONHbits.FRESET ||
		    C1FIFOCON2Hbits.FRESET ||
		    C1FIFOCON3Hbits.FRESET) {
			return;
		}
		nmea2000_status = NMEA2000_S_IDLE;
		break;
	case NMEA2000_S_IDLE:
		break;
	case NMEA2000_S_CLAIMING:
		if (C1TXQSTALbits.TXERR) {
			goto abort;
		}
		if (C1TXQSTALbits.TXQEIF == 0 || C1TXQCONHbits.TXREQ) {
			/* claim packet not sent yet */
			nmea2000_claim_date = 0;
		}
		break;
	}
}

static inline void
pic18can_init(void) {
	pic18can_config_mode();

	C1CONU = 0x10; /* TXQ en, STEF disable */
	C1CONH = 0x90; /* enable CAN, BRSDIS */
	C1CONL = 0x40; /* PXEDIS */

	/* setup fifos */
	C1FIFOBAT = C1FIFOBAU = 0;
	C1FIFOBAH = (unsigned int)(&ecanfifo[0]) >> 8;
	C1FIFOBAL = 0;

	/* transmit queue (control messages) */
	C1TXQCONT = CAN_TXSQIZE - 1;
	C1TXQCONU = 0x7f; /* unlimited retrans, pri 31 */
	C1TXQCONH = 0;
	C1TXQCONL = 0x80;

	/* fifo 1 (receive) */
	C1FIFOCON1T = CAN_FIFO1SIZE - 1;
	C1FIFOCON1U = 0;
	C1FIFOCON1H = 0;
	C1FIFOCON1L = 1; /* interrupt enable */

	/* fifo 2 (transmit single frames) */
	C1FIFOCON2T = CAN_FIFO2SIZE - 1;
	C1FIFOCON2U = 0x60; /* unlimited retrans */
	C1FIFOCON2H = 0;
	C1FIFOCON2L = 0x80;

#ifdef NEMA2000_USE_FAST_FRAME
	/* fifo 3 (transmit fast frames) */
	C1FIFOCON3T = CAN_FIFO3SIZE - 1;
	C1FIFOCON3U = 0x60; /* unlimited retrans */
	C1FIFOCON3H = 0;
	C1FIFOCON3L = 0x80;
#else 
	C1FIFOCON3T = C1FIFOCON3U = C1FIFOCON3H = C1FIFOCON3L = 0;
#endif

	/*
	 * filter 0 receives ISO broadcast messages
	 * 0xf000 <= PFN <= 0xffff
	 */
	C1MASK0L = 0xbc; /* sid 7-0 */
	C1MASK0H = 0; /* sid 8-10, eid 4-0 */
	C1MASK0U = 0; /* eid 5-12 */
	C1MASK0T = _C1FLTOBJ0T_EXIDE_MASK; /* also eid 13 - 17 */
	C1FLTOBJ0L = 0x3c; /* sid 7-0 */
	C1FLTOBJ0H = 0; /* sid 8-10, eid 4-0 */
	C1FLTOBJ0U = 0; /* eid 5-12 */
	C1FLTOBJ0T = _C1FLTOBJ0T_EXIDE_MASK; /* also eid 13 - 17 */

	/*
	 * filter 1 receives N2K broadcast messages
	 * PFN >= 0x10000
	 */
	C1MASK1L = 0xc0; /* sid 7-0 */
	C1MASK1H = 0; /* sid 8-10, eid 4-0 */
	C1MASK1U = 0; /* eid 5-12 */
	C1MASK1T = _C1FLTOBJ0T_EXIDE_MASK; /* also eid 13 - 17 */
	C1FLTOBJ1L = 0x40; /* sid 7-0 */
	C1FLTOBJ1H = 0; /* sid 8-10, eid 4-0 */
	C1FLTOBJ1U = 0; /* eid 5-12 */
	C1FLTOBJ1T = _C1FLTOBJ0T_EXIDE_MASK; /* also eid 13 - 17 */

	/* filter 2 receives messages addressed to global address */
	C1MASK2L = 0x80; /* sid 7-0 */
	C1MASK2H = 0; /* sid 8-10, eid 4-0 */
	C1MASK2U = 0xf8; /* eid 5-12 */
	C1MASK2T = _C1FLTOBJ0T_EXIDE_MASK | 0x07; /* also eid 13 - 17 */
	C1FLTOBJ2L = 0; /* sid 7-0 */
	C1FLTOBJ2H = 0; /* sid 8-10, eid 4-0 */
	C1FLTOBJ2U = (NMEA2000_ADDR_GLOBAL << 3) & 0xff; /* eid 5-12 */
	C1FLTOBJ2T = _C1FLTOBJ0T_EXIDE_MASK |
	    ((NMEA2000_ADDR_GLOBAL >> 5) & 0x1f); /* also eid 13 - 17 */
	/*
	 * filter 3 receives messages addressed to our address
	 * For now set to NMEA2000_ADDR_GLOBAL, will change when
	 * claim_address completes.
	 */
	C1MASK3L = 0x80; /* sid 7-0 */
	C1MASK3H = 0; /* sid 8-10, eid 4-0 */
	C1MASK3U = 0xf8; /* eid 5-12 */
	C1MASK3T = _C1FLTOBJ0T_EXIDE_MASK | 0x07; /* also eid 13 - 17 */
	C1FLTOBJ3L = 0; /* sid 7-0 */
	C1FLTOBJ3H = 0; /* sid 8-10, eid 4-0 */
	C1FLTOBJ3U = (NMEA2000_ADDR_GLOBAL << 3) & 0xff; /* eid 5-12 */
	C1FLTOBJ3T = _C1FLTOBJ0T_EXIDE_MASK |
	    ((NMEA2000_ADDR_GLOBAL >> 5) & 0x1f); /* also eid 13 - 17 */

	/* enable filters 0, 1, 2 for fifo 1 - filter 3 enabled later */
	C1FLTCON0L = 0x81;
	C1FLTCON0H = 0x81;
	C1FLTCON0U = 0x81;
	C1FLTCON0T = 0;
	C1FLTCON1L = C1FLTCON1H = C1FLTCON1U = C1FLTCON1T = 0;
	C1FLTCON2L = C1FLTCON2H = C1FLTCON2U = C1FLTCON2T = 0;

	/* setup baud rate */
	C1NBTCFGT = 0;
	C1NBTCFGU = NBTCFGU_uval;
	C1NBTCFGH = NBTCFGH_uval;
	C1NBTCFGL = NBTCFGL_uval;

	pic18can_normal_mode();

	C1INTT = 0;
	C1INTU = 0x2; /* RX interrupts enable */
}

void __interrupt(__irq(CAN), __low_priority, base(IVECT_BASE))
irql_pic18can(void)
{
	C1INTU = 0; /* main loop will check */
}

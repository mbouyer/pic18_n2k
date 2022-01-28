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

/* CAN backend for pic18 CAN modules (e.g. pic18f25k80) */

#include <pic18fregs.h>

#define PIC18_CAN_IRQ 0x12; /* enable: receive, transmit */

#define PIC18_TX_QUEUE_SIZE 16
#define PIC18_TX_QUEUE_MASK (PIC18_TX_QUEUE_SIZE - 1)

struct pic18_can_frame {
	unsigned char sidh;
	unsigned char sidl;
	unsigned char eidh;
	unsigned char eidl;
	unsigned char dlc;
	unsigned char data[8];
};
static struct pic18_can_frame pix18_tx_queue[PIC18_TX_QUEUE_SIZE];

static unsigned char pix18_tx_queue_cons;
static unsigned char pix18_tx_queue_prod;

static inline void
pic18can_config_mode(void)
{
	CANCON = 0x80;
	while ((CANSTAT & 0xe0) != 0x80)
		;
}

static inline void
pic18can_normal_mode(void)
{
	CANCON = 0x00;
	while ((CANSTAT & 0xe0) != 0x00)
		;
}

static inline void
pic18can_set_filter(unsigned char addr)
{
	PIE5 = 0;
	pic18can_config_mode();
        RXF3EIDH = addr;
	pic18can_normal_mode();
	PIE5 = PIC18_CAN_IRQ;
}

static void
pic18_startxmit_single()
{
	struct pic18_can_frame *txq = &pix18_tx_queue[pix18_tx_queue_cons];
	unsigned char *datareg = &TXB1D0;
	unsigned char i;

	if (TXB1CONbits.TXREQ) {
		printf("pic18_startxmit error\n");
		return;
	}
	if (pix18_tx_queue_cons == pix18_tx_queue_prod) {
		/* queue empty */
		TXBIEbits.TXB1IE = 0;
		return;
	}

	TXB1SIDH = txq->sidh;
	TXB1SIDL = txq->sidl;
	TXB1EIDH = txq->eidh;
	TXB1EIDL = txq->eidl;
	TXB1DLC = txq->dlc;
	for (i = 0; i < txq->dlc; i++)
		datareg[i] = txq->data[i];

	pix18_tx_queue_cons = (pix18_tx_queue_cons + 1) & PIC18_TX_QUEUE_MASK;
	TXBIEbits.TXB1IE = 1;
	TXB1CONbits.TXREQ = 1;
}

void
nmea2000_receive()
{
	while (COMSTATbits.NOT_FIFOEMPTY) {
		/* map receive buffer */
		unsigned char reg;

		reg = ECANCON & 0xe0;
		reg |= (CANCON & 0x0f) | 0x10;
		ECANCON = reg;

		rid.id = RXB0EIDL | ( RXB0EIDH << 8);
		rid.iso_pg =  (RXB0SIDL & 0x3) | ((RXB0SIDL & 0xe0) >> 3) |
		    (RXB0SIDH & 0x7) << 5;
		rid.page = (RXB0SIDH & 0x08) ? 1 : 0;
		rid.priority = (RXB0SIDH & 0xe0) >> 5;
		nmea2000_do_receive();

		RXB0CON = 0;
		PIR5bits.RXBnIF = 0;

	}
	PIE5bits.RXBnIE = 1;
}

void
nmea2000_intr()
{
	if (PIR5bits.TXBnIF && PIE5bits.TXBnIE) {
		if (TXBIEbits.TXB1IE && TXB1CONbits.TXREQ == 0)
			pic18_startxmit_single();

		PIR5bits.TXBnIF = 0;
	}
}

char
nmea2000_send_single_frame(__data struct nmea2000_msg *msg)
{
	unsigned char i;
	unsigned char new_tx_queue_prod;
	struct pic18_can_frame *txq;

	if (nmea2000_addr_status != ADDR_STATUS_OK || canbus_mute)
		return 0;

	new_tx_queue_prod = (pix18_tx_queue_prod + 1) & PIC18_TX_QUEUE_MASK;

	if (new_tx_queue_prod == pix18_tx_queue_cons) {
		/* queue full */
		return 0;
	}

	txq = &pix18_tx_queue[pix18_tx_queue_prod];
	txq->sidh = (msg->id.id >> 21);
	txq->sidl = ((msg->id.id >> 13) & 0xe0) | _EXIDE | ((msg->id.id >> 16) & 0x3);
	txq->eidh = (msg->id.id >> 8);
	txq->eidl = nmea2000_addr;
	txq->dlc = msg->dlc;
	for (i = 0; i < msg->dlc; i++)
		txq->data[i] = msg->data[i];

	PIE5bits.TXBnIE = 0;
	pix18_tx_queue_prod = new_tx_queue_prod;
	if (TXBIEbits.TXB1IE == 0) {
		pic18_startxmit_single();
	}
	PIE5bits.TXBnIE = 1;
	return 1;
}

#ifdef NEMA2000_USE_FAST_FRAME

char
nmea2000_send_fast_frame(__data struct nmea2000_msg *msg, unsigned char id)
{
	unsigned char new_tx_queue_prod;
	struct pic18_can_frame *txq;
	unsigned char i, j, n;
	unsigned char len = msg->dlc;
	const char *data = msg->data;

	if (nmea2000_addr_status != ADDR_STATUS_OK || canbus_mute)
		return 0;

	for (n  = 0; len > 0; n++) {
		new_tx_queue_prod = (pix18_tx_queue_prod + 1) & PIC18_TX_QUEUE_MASK;

		if (new_tx_queue_prod == pix18_tx_queue_cons) {
			/* queue full */
			return 0;
		}

		txq = &pix18_tx_queue[pix18_tx_queue_prod];
		txq->sidh = (msg->id.id >> 21);
		txq->sidl = ((msg->id.id >> 13) & 0xe0) | _EXIDE | ((msg->id.id >> 16) & 0x3);
		txq->eidh = (msg->id.id >> 8);
		txq->eidl = nmea2000_addr;
		txq->data[0] = (id << 5) | n ;
		if (n == 0) {
			txq->data[1] = len;
			for (i = 0; i < 6; i++)
				txq->data[i+2] = data[i];
			len -= 6;
			data += 6;
			txq->dlc = 8;
		} else {
			j = len;
			if (j > 7)
				j = 7;
			for (i = 0; i < j; i++)
				txq->data[i+1] = data[i];
			len -= j;
			data += j;
			txq->dlc = j + 1;
		}

		PIE5bits.TXBnIE = 0;
		pix18_tx_queue_prod = new_tx_queue_prod;
		if (TXBIEbits.TXB1IE == 0) {
			pic18_startxmit_single();
		}
		PIE5bits.TXBnIE = 1;
	}
	return 1;
}
#endif /* NEMA2000_USE_FAST_FRAME */

static char
nmea2000_send_control(struct nmea2000_msg *msg)
{
	char *datareg = &TXB2D0;
	unsigned char i;

	if (TXB2CONbits.TXREQ != 0)
		return 0;

	TXB2SIDH = (msg->id.id >> 21);
	TXB2SIDL = ((msg->id.id >> 13) & 0xe0) | _EXIDE | ((msg->id.id >> 16) & 0x3);
	TXB2EIDH = (msg->id.id >> 8);
	TXB2EIDL = nmea2000_addr;
	TXB2DLC = msg->dlc;
	for (i = 0; i < msg->dlc; i++)
		datareg[i] = msg->data[i];

	TXB2CONbits.TXREQ = 1;
	return 1;
}

static inline void
pic18can_init(void) {
	pic18can_config_mode();

	ECANCON = 0x10;
	ECANCONbits.MDSEL = 2;

	CIOCON = 0;
	CIOCONbits.CLKSEL = 1;
	CIOCONbits.ENDRHI = 1;
#if defined(CAN_PORTB)
	TRISBbits.TRISB3 = 1;
#elif defined(CAN_PORTC)
	TRISCbits.TRISC7 = 1;
#else
#error	"CAN port not defined"
#endif

	/*
	 * filter 0/mask 0 receives ISO broadcast messages
	 * 0xf000 <= PFN <= 0xffff
	 */
        RXM0SIDH = 0x07;
        RXM0SIDL = 0x80 | _RXM0SIDL_EXIDEN;
        RXM0EIDH = 0x00;
        RXM0EIDL = 0x00;
        RXF0SIDH = 0x07;
        RXF0SIDL = 0x80 | _RXF0SIDL_EXIDEN;
        RXF0EIDH = 0x00;
        RXF0EIDL = 0x00;

	/*
	 * filter 1/mask 1 receives N2K broadcast messages
	 * PFN >= 0x10000
	 */
        RXM1SIDH = 0x08;
        RXM1SIDL = 0x00 | _RXM1SIDL_EXIDEN;
        RXM1EIDH = 0x00;
        RXM1EIDL = 0x00;
        RXF1SIDH = 0x08;
        RXF1SIDL = 0x00 | _RXF1SIDL_EXIDEN;
        RXF1EIDH = 0x00;
        RXF1EIDL = 0x00;

	/*
	 * filter 2/mask 15 receives messages addressed to global address
	 * filter 3/mask 15 receives messages addressed to our address
	 */
        RXF15SIDH = 0x00;
        RXF15SIDL = 0x00 | _RXF1SIDL_EXIDEN;
        RXF15EIDH = 0xff;
        RXF15EIDL = 0x00;

        RXF2SIDH = 0x00;
        RXF2SIDL = 0x00 | _RXF1SIDL_EXIDEN;
        RXF2EIDH = NMEA2000_ADDR_GLOBAL;
        RXF2EIDL = 0x00;

        RXF3SIDH = 0x00;
        RXF3SIDL = 0x00 | _RXF1SIDL_EXIDEN;
        RXF3EIDH = NMEA2000_ADDR_GLOBAL; /* will change later */
        RXF3EIDL = 0x00;

	MSEL0 = 0xa4; /* b10100100 */

	/* All filters associated to RXB0 */
	RXFBCON0 = 0;
	RXFBCON1 = 0;

	/* enable filters 0-3 */
	RXFCON0 = 0x0f;
	RXFCON1 = 0x00;

	/* user-supplied values */
	BRGCON1 = BRGCON1_uval;
	BRGCON2 = BRGCON2_uval;
	BRGCON3 = BRGCON3_uval;

	/* All TX/RX buffers are for receive */
	BSEL0 = 0;

	/* setup receive buffers */
	RXB0CON = 0;
	RXB1CON = 0;
	B0CON = 0;
	B1CON = 0;
	B2CON = 0;
	B3CON = 0;
	B4CON = 0;
	B5CON = 0;

	/* setup transmit buffers, from low to hi priority:
	 * TX0 for fast (multi-frame) messages
	 * TX1 for normal messages
	 * TX2 for management
	 */
	TXB0CON = 0x0;
	TXB1CON = 0x1;
	TXB2CON = 0x2;

	pic18can_normal_mode();

	/* setup interrupts */
	PIR5 = 0; /* clear interrupts */
	IPR5 = 0; /* everything is low priority */
	BIE0 = 0xff; /* all receive buffers interrupts */
	TXBIE = 0; /* no transmit interrupt */
	PIE5 = PIC18_CAN_IRQ;

	pix18_tx_queue_cons = pix18_tx_queue_prod = 0;
}

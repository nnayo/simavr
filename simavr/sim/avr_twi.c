/*
	avr_twi.c

	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>
	Copyright 2014       Yann Gouy <yann_gouy@yahoo.fr>

 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "avr_twi.h"

#include <stdlib.h>	// malloc(), free()


/*
 * This block respectfully nicked straight out from the Atmel sample
 * code for AVR315. Typos and all.
 * There is no copyright notice on the original file.
 */
/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master status codes
#define TWI_START					0x08  // START has been transmitted
#define TWI_REP_START				0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST				0x38  // Arbitration lost

// TWI Master Transmitter status codes
#define TWI_MTX_ADR_ACK				0x18  // SLA+W has been transmitted and ACK received
#define TWI_MTX_ADR_NACK			0x20  // SLA+W has been transmitted and NACK received
#define TWI_MTX_DATA_ACK			0x28  // Data byte has been transmitted and ACK received
#define TWI_MTX_DATA_NACK			0x30  // Data byte has been transmitted and NACK received

// TWI Master Receiver status codes
#define TWI_MRX_ADR_ACK				0x40  // SLA+R has been transmitted and ACK received
#define TWI_MRX_ADR_NACK			0x48  // SLA+R has been transmitted and NACK received
#define TWI_MRX_DATA_ACK			0x50  // Data byte has been received and ACK transmitted
#define TWI_MRX_DATA_NACK			0x58  // Data byte has been received and NACK transmitted

// TWI Slave Transmitter status codes
#define TWI_STX_ADR_ACK				0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST	0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK			0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK			0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE	0xC8  // Last data byte in TWDR has been transmitted (TWEA = �0�); ACK has been received

// TWI Slave Receiver status codes
#define TWI_SRX_ADR_ACK				0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK				0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST	0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK		0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK		0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK		0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK		0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART		0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE				0xF8  // No relevant state information available; TWINT = �0�
#define TWI_BUS_ERROR				0x00  // Bus error due to an illegal START or STOP condition

// number of TWI bus clock ticks
#define AVR_TWI_NO_CYCLE	0		// nb cycles for some specific functions
#define AVR_TWI_ACK_CYCLES	1		// nb cycles for sending an ack
#define AVR_TWI_NACK_CYCLES	1		// nb cycles for sending a nack
#define AVR_TWI_START_CYCLES	2		// nb cycles for sending a start
#define AVR_TWI_STOP_CYCLES	2		// nb cycles for sending a stop
#define AVR_TWI_ADDR_CYCLES	8		// nb cycles for sending an address
#define AVR_TWI_DATA_CYCLES	8		// nb cycles for sending a data


#define _TWCR_COND_0000	(!twcr.twsta && !twcr.twsto && !twcr.twint && !twcr.twea)

#define _TWCR_COND_0010	(!twcr.twsta && !twcr.twsto &&  twcr.twint && !twcr.twea)
#define _TWCR_COND_0011	(!twcr.twsta && !twcr.twsto &&  twcr.twint &&  twcr.twea)
#define _TWCR_COND_001x	(!twcr.twsta && !twcr.twsto &&  twcr.twint &&  1		)

#define _TWCR_COND_00x1	(!twcr.twsta && !twcr.twsto &&  1		  &&  twcr.twea)
#define _TWCR_COND_00xx	(!twcr.twsta && !twcr.twsto &&  1		  &&  1		)

#define _TWCR_COND_011x	(!twcr.twsta &&  twcr.twsto &&  twcr.twint &&  1		)

#define _TWCR_COND_1010	( twcr.twsta && !twcr.twsto &&  twcr.twint && !twcr.twea)
#define _TWCR_COND_101x	( twcr.twsta && !twcr.twsto &&  twcr.twint &&  1		)

#define _TWCR_COND_10x1	( twcr.twsta && !twcr.twsto &&  1		  &&  twcr.twea)

#define _TWCR_COND_111x	( twcr.twsta &&  twcr.twsto &&  twcr.twint &&  1		)

#define _TWCR_COND_x01x	( 1		  && !twcr.twsto &&  twcr.twint &&  1		)

#define _TWCR_COND_x0x0	( 1		  && !twcr.twsto &&  1		  && !twcr.twea)
#define _TWCR_COND_x0x1	( 1		  && !twcr.twsto &&  1		  &&  twcr.twea)

#define _TWCR_COND_xxx0	( 1		  &&  1		  &&  1		  && !twcr.twea)
#define _TWCR_COND_xxx1	( 1		  &&  1		  &&  1		  &&  twcr.twea)
#define _TWCR_COND_xxxx	( 1		  &&  1		  &&  1		  &&  1		)


typedef struct avr_twi_set_state_t {
	avr_twi_t* twi;
	uint8_t twsr;
	uint8_t bus_state:1;
	uint8_t start_pending:1;
	uint8_t raise_interrupt:1;
	uint8_t msg_ok:1;
	struct avr_twi_msg_irq_t msg;
} avr_twi_set_state_t;


// activate debug traces
#define AVR_TWI_DEBUG

#ifdef AVR_TWI_DEBUG
# define AVR_TWI_TRACE(avr, ...)	AVR_TRACE(avr, __VA_ARGS__)

# define BRIGTH_COLOR	"\x1b[95m"
# define NORMAL_COLOR	"\x1b[0m"

static char* msg2chr[] = {
	BRIGTH_COLOR"0"NORMAL_COLOR,
	BRIGTH_COLOR"["NORMAL_COLOR,
	BRIGTH_COLOR"@"NORMAL_COLOR,
	BRIGTH_COLOR"D"NORMAL_COLOR,
	BRIGTH_COLOR"+"NORMAL_COLOR,
	BRIGTH_COLOR"-"NORMAL_COLOR,
	BRIGTH_COLOR"w"NORMAL_COLOR,
	BRIGTH_COLOR"]"NORMAL_COLOR
};

#else
# define AVR_TWI_TRACE(avr, ...)
#endif


// update twi state reg on time-out
static avr_cycle_count_t
_avr_twi_state_update(
		struct avr_t * avr,
		avr_cycle_count_t when,
		void * param)
{
	struct avr_twi_set_state_t* new_state = (struct avr_twi_set_state_t *)param;
	avr_twi_t * p = (avr_twi_t *)new_state->twi;

	AVR_TWI_TRACE(p->io.avr, "TWI:[\x1b[94m%s\x1b[0m] %s             (t=%d) --> (bus:%d p:%d TWSR:0x%02x) i:%d", p->io.avr->tag_name, __func__, when, new_state->bus_state, new_state->start_pending, new_state->twsr, new_state->raise_interrupt);

	if (new_state->msg_ok) {
#ifdef AVR_TWI_DEBUG
		struct avr_twi_msg_irq_t msg = new_state->msg;
#endif
		AVR_TWI_TRACE(avr, "\tmsg %s  addr 0x%02x+%c / data 0x%02x\n", msg2chr[msg.bus.msg], msg.bus.data >> 1, msg.bus.data & 0x01 ? 'R' : 'W', msg.bus.data);
		avr_raise_irq(p->io.irq + TWI_IRQ_OUTPUT, new_state->msg.v);
	}
	else {
		AVR_TWI_TRACE(avr, "\n");
	}

	p->bus_state = new_state->bus_state;
	p->start_pending = new_state->start_pending;

	avr_regbit_setto_raw(p->io.avr, p->twsr, new_state->twsr);
	avr_raise_irq(p->io.irq + TWI_IRQ_STATUS, new_state->twsr);
	if (new_state->raise_interrupt)
		avr_raise_interrupt(p->io.avr, &p->twi);

	free(new_state);

	return 0;
}

/*
 * This triggers a timer whose duration is a multiple
 * of 'twi' clock cycles, which should be derived from the prescaler
 * (100khz, 400khz etc).
 */

#define TWSR_UNCHG	-1

#define BUS_INACT	0
#define BUS_UNCHG	-1
#define BUS_ACT		1

#define START_NO	0
#define START_UNCHG	-1
#define START_PEND	1

#define WO_INT		0
#define W_INT		1

static void
_avr_twi_delay_state(
		avr_twi_t * twi,
		int twi_cycles,
		uint8_t twsr,				// -1: unchanged
		avr_twi_msg_irq_t* msg,
		uint8_t bus_state,			// -1: unchanged
		uint8_t start_pending,		// -1: unchanged
		uint8_t raise_interrupt)
{
	struct avr_twi_set_state_t* next_state;

	// allocate and set the next state
	next_state = (struct avr_twi_set_state_t*)malloc(sizeof(struct avr_twi_set_state_t));

	next_state->twi = twi;
	next_state->twsr = (twsr == (uint8_t)-1) ? twi->io.avr->data[twi->r_twsr] : twsr;

	if (msg) {
		next_state->msg_ok = 1;
		next_state->msg = *msg;
	}
	else {
		next_state->msg_ok = 0;
	}

	next_state->bus_state = (bus_state == (uint8_t)-1) ? twi->bus_state : bus_state;
	next_state->start_pending = (start_pending == (uint8_t)-1) ? twi->start_pending : start_pending;
	next_state->raise_interrupt = raise_interrupt;

	// calculate clock rate, convert to cycles, and use that to set a time-out
	int twps10 = twi->io.avr->data[twi->r_twsr] & 0x03;
	int prescale = 1 << (2 * twps10);
	int scale = 16 + 2 * twi->io.avr->data[twi->r_twbr] * prescale;
	int cycles = twi_cycles * scale;

	avr_cycle_timer_register(twi->io.avr, cycles, _avr_twi_state_update, next_state);

	AVR_TWI_TRACE(twi->io.avr, 
		"TWI:[\x1b[94m%s\x1b[0m] %s in %d cycle(s)(t=%d) ==> (bus:%d p:%d TWSR:0x%02x)",
		twi->io.avr->tag_name, __func__, twi_cycles, twi->io.avr->cycle + cycles,
		twi->bus_state, twi->start_pending, next_state->twsr
		);

	if (msg) {
		AVR_TWI_TRACE(twi->io.avr, " msg %s\n", msg2chr[msg->bus.msg]);
	}
	else {
		AVR_TWI_TRACE(twi->io.avr, "\n");
	}
#ifdef AVR_TWI_DEBUG
	if (twsr == TWI_BUS_ERROR) {
		// segfault the code
		void(*f)(void) = NULL;
		(*f)();
	}
#endif
}

// update twi status reg on time-out
static avr_cycle_count_t
avr_twi_config_stop_timer(
		struct avr_t * avr,
		avr_cycle_count_t when,
		void * param)
{
	avr_twi_t * p = (avr_twi_t *)param;

	avr_regbit_clear(p->io.avr, p->twsto);
	AVR_TWI_TRACE(p->io.avr, "TWI:[\x1b[94m%s\x1b[0m] %s\n", p->io.avr->tag_name, __func__);

	return 0;
}

static void
_avr_twi_delay_config_stop(
		avr_twi_t * twi,
		int twi_cycles)
{
	// calculate clock rate, convert to cycles, and use that to set a time-out
	int twps10 = twi->io.avr->data[twi->r_twsr] & 0x03;
	int prescale = 1 << (2 * twps10);
	int scale = 16 + 2 * twi->io.avr->data[twi->r_twbr] * prescale;
	int cycles = twi_cycles * scale;

	avr_cycle_timer_register(twi->io.avr, cycles, avr_twi_config_stop_timer, twi);

	AVR_TWI_TRACE(twi->io.avr, "TWI:[\x1b[94m%s\x1b[0m] %s in %d cycle(s)\n", twi->io.avr->tag_name, __func__, twi_cycles);
}


typedef struct twcr_t {
	uint8_t twint:1;
	uint8_t twea:1;
	uint8_t twsta:1;
	uint8_t twsto:1;
	uint8_t twen:1;
} _twcr_t;


// TWI_BUS_ERROR	0x00
static void _avr_twi_fsm_bus_error(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// soft is resetting TWI?
		if (_TWCR_COND_011x) {
			avr_regbit_clear(avr, p->twsto);
			avr_core_watch_write(avr, p->r_twsr, TWI_NO_STATE);
			return;
		}
	}

	// bus message ?
	if (link) {
	}

#ifdef AVR_TWI_DEBUG
	// segfault the code
	void(*f)(void) = NULL;
	(*f)();
#endif
}

// TWI_START                 0x08
static void _avr_twi_fsm_start(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send SLA+W or SLA+R
		if (_TWCR_COND_001x) {
			uint8_t slave_addr = avr->data[p->r_twdr];
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ADDR, slave_addr);
			_avr_twi_delay_state(p, AVR_TWI_ADDR_CYCLES, TWSR_UNCHG, &msg, BUS_UNCHG, START_UNCHG, WO_INT);

			// general call handling
			if ((slave_addr & 0xfe) == 0x00) {
				p->gencall = 1;
			}
			return;
		}
	}

	// bus message ?
	if (link) {
		int read = (link->bus.addr & 1) == 1;
		int write = (link->bus.addr & 1) == 0;
		int gencall = link->bus.addr == 0x00;
		int ack = link->bus.msg == TWI_MSG_ACK;
		int nack = link->bus.msg == TWI_MSG_NACK;

		// SLA+W acked ?
		if (write && ack) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MTX_ADR_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// SLA+W nacked ?
		if (write && nack) {
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_MTX_ADR_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// SLA+R acked ?
		if (read && ack) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MRX_ADR_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// SLA+R nacked ?
		if (read && nack) {
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_MRX_ADR_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// gen call acked and not already acked ?
		if (gencall && ack && !p->gencall) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MTX_ADR_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			p->gencall = 1;
			return;
		}

		// gen call nacked and never acked ?
		if (gencall && nack && !p->gencall) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MTX_ADR_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0x10
static void _avr_twi_fsm_rep_start(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send SLA+W or SLA+R
		if (_TWCR_COND_001x) {
			uint8_t slave_addr = avr->data[p->r_twdr];
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ADDR, slave_addr);
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWSR_UNCHG, &msg, BUS_UNCHG, START_UNCHG, WO_INT);

			// general call handling
			if (slave_addr == 0x00) {
				p->gencall = 1;
			}
			return;
		}
	}

	// bus message ?
	if (link) {
		int read = (link->bus.addr & 1) == 1;
		int write = (link->bus.addr & 1) == 0;
		int gencall = link->bus.addr == 0x00;
		int ack = link->bus.msg == TWI_MSG_ACK;
		int nack = link->bus.msg == TWI_MSG_NACK;

		// SLA+W acked ?
		if (write && ack) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MTX_ADR_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// SLA+W nacked ?
		if (write && nack) {
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_MTX_ADR_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// SLA+R acked ?
		if (read && ack) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MRX_ADR_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// SLA+R nacked ?
		if (read && nack) {
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_MRX_ADR_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// gen call acked and not already acked ?
		if (gencall && ack && !p->gencall) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MTX_ADR_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			p->gencall = 1;
			return;
		}

		// gen call nacked and never acked ?
		if (gencall && nack && !p->gencall) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MTX_ADR_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_MTX_ADR_ACK		0x18
static void _avr_twi_fsm_mtx_adr_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send data
		if (_TWCR_COND_001x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_DATA, avr->data[p->r_twdr]);
			_avr_twi_delay_state(p, AVR_TWI_DATA_CYCLES, TWSR_UNCHG, &msg, BUS_UNCHG, START_UNCHG, WO_INT);

			// in case of gencall, at least 1 slave shall ack
			p->gencall = 0;
			return;
		}
	}

	// bus message ?
	if (link) {
		// data acked and not already gencall acked ?
		if (link->bus.msg == TWI_MSG_ACK && !p->gencall) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_MTX_DATA_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			p->gencall = 1;
			return;
		}

		// data nacked and never gencall acked?
		if (link->bus.msg == TWI_MSG_NACK && !p->gencall) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_MTX_DATA_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0x20
static void _avr_twi_fsm_mtx_adr_nack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send restart
		if (_TWCR_COND_101x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_START, 0);
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_REP_START, &msg, BUS_ACT, START_NO, WO_INT);
			return;
		}

		// send stop
		if (_TWCR_COND_011x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, &msg, BUS_INACT, START_UNCHG, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}

		// send stop then start
		if (_TWCR_COND_111x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, &msg, BUS_ACT, START_PEND, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}
	}

	// bus message ?
	if (link) {
		if (link->bus.msg == TWI_MSG_NULL) {
			// null ignored
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0x28
static void _avr_twi_fsm_mtx_data_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send data
		if (_TWCR_COND_001x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_DATA, avr->data[p->r_twdr]);
			_avr_twi_delay_state(p, AVR_TWI_DATA_CYCLES, TWI_MTX_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, WO_INT);
			return;
		}

		// send RESTART
		if (_TWCR_COND_101x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_START, 0);
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_REP_START, &msg, BUS_ACT, START_NO, W_INT);
			return;
		}

		// send STOP
		if (_TWCR_COND_011x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, &msg, BUS_INACT, START_UNCHG, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}
	}

	// bus message ?
	if (link) {
		// data acked ?
		if (link->bus.msg == TWI_MSG_ACK) {
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MTX_DATA_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// data nacked ?
		if (link->bus.msg == TWI_MSG_NACK) {
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_MTX_DATA_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		if (link->bus.msg == TWI_MSG_NULL) {
			// null ignored
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0x30
static void _avr_twi_fsm_mtx_data_nack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send data
		if (_TWCR_COND_001x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_DATA, 0);
			_avr_twi_delay_state(p, AVR_TWI_DATA_CYCLES, TWI_MTX_DATA_NACK, &msg, BUS_UNCHG, START_NO, W_INT);
			return;
		}

		// send restart
		if (_TWCR_COND_101x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_START, 0);
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_START, &msg, BUS_UNCHG, START_NO, W_INT);
			return;
		}

		// send stop
		if (_TWCR_COND_011x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, &msg, BUS_INACT, START_UNCHG, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}

		// send stop then start
		if (_TWCR_COND_111x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, &msg, BUS_UNCHG, START_PEND, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}
	}

	// bus message ?
	if (link) {
		// receiving a null (as stop response)
		if (link->bus.msg == TWI_MSG_NULL) {
			// ignore it
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0x38
static void _avr_twi_fsm_arb_lost(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// release the bus
		if (_TWCR_COND_001x) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_UNCHG, START_NO, W_INT);
			return;
		}

		// start requested
		if (_TWCR_COND_101x) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_UNCHG, START_PEND, W_INT);
			return;
		}
	}

	// bus message ?
	if (link) {
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_MRX_ADR_ACK			0x40
static void _avr_twi_fsm_mrx_adr_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		if (_TWCR_COND_001x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_CLK, 0);
			_avr_twi_delay_state(p, AVR_TWI_DATA_CYCLES, TWI_MRX_ADR_ACK, &msg, BUS_UNCHG, START_UNCHG, WO_INT);
			return;
		}
	}

	// bus message ?
	if (link) {
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_0011) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MRX_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_0010) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_NACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_MRX_DATA_NACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_MRX_ADR_NACK           0x48
static void _avr_twi_fsm_mrx_adr_nack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send stop
		if (_TWCR_COND_011x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, &msg, BUS_INACT, START_UNCHG, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}

	}

	// bus message ?
	if (link) {
		// response for stop
		if (link->bus.msg == TWI_MSG_NULL) {
			// ignore it
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_MRX_DATA_ACK			0x50
static void _avr_twi_fsm_mrx_data_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		if (_TWCR_COND_001x) {
			// send the clock
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_CLK, avr->data[p->r_twdr]);
			_avr_twi_delay_state(p, AVR_TWI_DATA_CYCLES, TWI_MRX_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, WO_INT);
			return;
		}

	}

	// bus message ?
	if (link) {
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_0011) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ACK, avr->data[p->r_twdr]);
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_MRX_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_0010) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_NACK, avr->data[p->r_twdr]);
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_MRX_DATA_NACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_MRX_DATA_NACK		0x58
static void _avr_twi_fsm_mrx_data_nack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// repeated start
		if (_TWCR_COND_101x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_START, 0);
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_START, &msg, BUS_ACT, START_NO, W_INT);
			return;
		}

		// stop
		if (_TWCR_COND_011x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, &msg, BUS_INACT, START_UNCHG, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}

		// stop then start
		if (_TWCR_COND_101x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_STOP, 0);
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, &msg, BUS_UNCHG, START_PEND, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}
	}

	// bus message ?
	if (link) {
		if (link->bus.msg == TWI_MSG_NULL && _TWCR_COND_xxxx) {
			// simply ignored
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0xa8
static void _avr_twi_fsm_stx_adr_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		if (_TWCR_COND_x01x) {
			// data is sent on clock
			return;
		}
	}

	// bus message ?
	if (link) {
		// clock received
		if (link->bus.msg == TWI_MSG_CLK && _TWCR_COND_x01x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_DATA, avr->data[p->r_twdr]);
			avr_raise_irq(p->io.irq + TWI_IRQ_OUTPUT, msg.v);
			return;
		}

		// data acked
		if (link->bus.msg == TWI_MSG_ACK) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_STX_DATA_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// data nacked
		if (link->bus.msg == TWI_MSG_NACK) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_STX_DATA_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0xb0
static void _avr_twi_fsm_stx_adr_nack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send data
		if (_TWCR_COND_x01x) {
			// data is sent on clock
			return;
		}
	}

	// bus message ?
	if (link) {
		// clock received
		if (link->bus.msg == TWI_MSG_CLK && _TWCR_COND_x01x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_DATA, avr->data[p->r_twdr]);
			avr_raise_irq(p->io.irq + TWI_IRQ_OUTPUT, msg.v);
			return;
		}

		// data acked
		if (link->bus.msg == TWI_MSG_ACK) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_STX_DATA_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// data nacked
		if (link->bus.msg == TWI_MSG_NACK) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_STX_DATA_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0xb8
static void _avr_twi_fsm_stx_data_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// send data
		if (_TWCR_COND_x01x) {
			// data is sent on clock
			return;
		}
	}

	// bus message ?
	if (link) {
		// clock received
		if (link->bus.msg == TWI_MSG_CLK && _TWCR_COND_x01x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_DATA, avr->data[p->r_twdr]);
			avr_raise_irq(p->io.irq + TWI_IRQ_OUTPUT, msg.v);
			return;
		}

		// data acked
		if (link->bus.msg == TWI_MSG_ACK) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_STX_DATA_ACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// data nacked
		if (link->bus.msg == TWI_MSG_NACK) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_STX_DATA_NACK, NULL, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0xc0
static void _avr_twi_fsm_stx_data_nack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// reset fsm
		if (_TWCR_COND_001x) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_UNCHG, WO_INT);
			return;
		}

		// start pending
		if (_TWCR_COND_101x) {
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_NO_STATE, NULL, BUS_ACT, START_UNCHG, WO_INT);
			return;
		}
	}

	// bus message ?
	if (link) {
		// stop
		if (link->bus.msg == TWI_MSG_STOP) {
			_avr_twi_delay_state(p, AVR_TWI_STOP_CYCLES, TWI_NO_STATE, NULL, BUS_INACT, START_UNCHG, WO_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0xc8
static void _avr_twi_fsm_stx_ack_last_byte(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
	}

	// bus message ?
	if (link) {
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_SRX_ADR_ACK		0x60
static void _avr_twi_fsm_srx_adr_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		if (_TWCR_COND_x01x) {
			// handling done below on data message
			return;
		}
	}

	// bus message ?
	if (link) {
		// data to ack
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_xxx1) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_ADR_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// data to nack
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_xxx0) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_NACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_SRX_ADR_DATA_NACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_SRX_ADR_ACK_M_ARB_LOST		0x68
static void _avr_twi_fsm_srx_adr_ack_m_arb_lost(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		if (_TWCR_COND_x01x) {
			// handling done below on data message
			return;
		}
	}

	// bus message ?
	if (link) {
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_0011) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ACK, avr->data[p->r_twdr]);
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_ADR_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_0010) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_NACK, avr->data[p->r_twdr]);
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_SRX_ADR_DATA_NACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_SRX_GEN_ACK		0x70
static void _avr_twi_fsm_srx_gen_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		if (_TWCR_COND_x01x) {
			// handling done below on data message
			return;
		}
	}

	// bus message ?
	if (link) {
		// data to ack
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_x0x1) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_GEN_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// data to nack
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_x0x0) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_NACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_SRX_GEN_DATA_NACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0x78
static void _avr_twi_fsm_srx_gen_ack_m_arb_lost(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
	}

	// bus message ?
	if (link) {
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_SRX_ADR_DATA_ACK		0x80
static void _avr_twi_fsm_srx_adr_data_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		if (_TWCR_COND_x01x) {
			// handling done below on data message
			return;
		}
	}

	// bus message ?
	if (link) {
		// data to ack
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_xxx1) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_ADR_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// data to nack
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_xxx0) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_NACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_SRX_ADR_DATA_NACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// receiving a restart
		if (link->bus.msg == TWI_MSG_START && _TWCR_COND_xxx1) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_SRX_STOP_RESTART, NULL, BUS_ACT, START_UNCHG, W_INT);
			return;
		}

		// receiving a stop
		if (link->bus.msg == TWI_MSG_STOP && _TWCR_COND_xxx1) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_SRX_STOP_RESTART, NULL, BUS_INACT, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_SRX_ADR_DATA_NACK		0x88
static void _avr_twi_fsm_srx_adr_data_nack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// switched to not addressed slave mode, no recognition of SLA or GCA
		if (_TWCR_COND_0010) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_UNCHG, W_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}

		// switched to not addressed slave mode
		if (_TWCR_COND_0011) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_UNCHG, W_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}
	}

	// bus message ?
	if (link) {
		int start = link->bus.msg == TWI_MSG_START;
		int stop = link->bus.msg == TWI_MSG_STOP;

		// receiving a restart
		if (start && _TWCR_COND_xxx1) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_SRX_STOP_RESTART, NULL, BUS_ACT, START_UNCHG, W_INT);
			return;
		}

		// receiving a stop, the bus is freed, send a start
		if (stop && _TWCR_COND_101x) {
			p->bus_state = 0;
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_START, 0);
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_START, &msg, BUS_ACT, START_NO, W_INT);
			return;
		}

		// receiving a stop and no action pending
		if (stop && _TWCR_COND_001x) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_NO, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_NO_CYCLE);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_SRX_GEN_DATA_ACK		0x90
static void _avr_twi_fsm_srx_gen_data_ack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		if (_TWCR_COND_x01x) {
			// handling done below on data message
			return;
		}
	}

	// bus message ?
	if (link) {
		// data to ack
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_xxx1) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_ACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_GEN_DATA_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// data to nack
		if (link->bus.msg == TWI_MSG_DATA && _TWCR_COND_xxx0) {
			avr_core_watch_write(avr, p->r_twdr, link->bus.data);
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_NACK, 0);
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_SRX_GEN_DATA_NACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}

		// receiving a restart
		if (link->bus.msg == TWI_MSG_START && _TWCR_COND_xxx1) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_SRX_STOP_RESTART, NULL, BUS_ACT, START_UNCHG, W_INT);
			return;
		}

		// receiving a stop
		if (link->bus.msg == TWI_MSG_STOP && _TWCR_COND_xxx1) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_SRX_STOP_RESTART, NULL, BUS_INACT, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0x98
static void _avr_twi_fsm_srx_gen_data_nack(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// switched to not addressed slave mode, no recognition of SLA or GCA
		if (_TWCR_COND_0010) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_UNCHG, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}

		// switched to not addressed slave mode
		if (_TWCR_COND_0011) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_UNCHG, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}

		// switched to not addressed slave mode, no recognition of SLA or GCA, start pending
		if (_TWCR_COND_1010) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_PEND, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}

		// switched to not addressed slave mode, start pending
		if (_TWCR_COND_0011) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_PEND, WO_INT);
			_avr_twi_delay_config_stop(p, AVR_TWI_STOP_CYCLES);
			return;
		}
	}

	// bus message ?
	if (link) {
		// receiving a stop
		if (link->bus.msg == TWI_MSG_STOP && _TWCR_COND_001x) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_INACT, START_UNCHG, WO_INT);
			return;
		}

		if (link->bus.msg == TWI_MSG_STOP && _TWCR_COND_101x) {
			avr_twi_msg_irq_t msg;
			msg = avr_twi_irq_msg(TWI_MSG_START, 0);
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_START, &msg, BUS_ACT, START_NO, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// 0xa0
static void _avr_twi_fsm_srx_stop_restart(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
	}

	// bus message ?
	if (link) {
		// receiving a start
		if (link->bus.msg == TWI_MSG_START && _TWCR_COND_xxxx) {
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_NO_STATE, NULL, BUS_ACT, START_UNCHG, WO_INT);
			return;
		}

		// receiving a stop
		if (link->bus.msg == TWI_MSG_STOP && _TWCR_COND_xxxx) {
			_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_NO_STATE, NULL, BUS_INACT, START_UNCHG, WO_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

static void _avr_twi_fsm_unknown(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
#ifdef AVR_TWI_DEBUG
	avr_t * avr = p->io.avr;
#endif
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
	}

	// bus message ?
	if (link) {
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// TWI_NO_STATE			0xf8
static void _avr_twi_fsm_no_state(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link)
{
	avr_t * avr = p->io.avr;
	AVR_TWI_TRACE(avr, "%s\n", __func__);

	// register access ?
	if (!link) {
		// reset or activate TWI
		if (_TWCR_COND_00xx) {
			p->bus_state = 0;
			return;
		}

		// request for START ?
		if (_TWCR_COND_101x) {
			if (p->bus_state == 0) {
				avr_twi_msg_irq_t msg;
				msg = avr_twi_irq_msg(TWI_MSG_START, 0);
				_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_START, &msg, BUS_ACT, START_NO, W_INT);
				return;
			}
			else {
				p->start_pending = 1;
				return;
			}
		}
	}

	// bus message ?
	if (link) {
		// START received ?
		if (link->bus.msg == TWI_MSG_START && _TWCR_COND_xxxx) {
			p->bus_state = 1;
			return;
		}

		// bus inactive ?
		if (!p->bus_state) {
			return;
		}

		// STOP received ?
		if (link->bus.msg == TWI_MSG_STOP && _TWCR_COND_xxxx) {
			p->bus_state = 0;
			// start requested
			if (_TWCR_COND_10x1) {
				avr_twi_msg_irq_t msg;
				msg = avr_twi_irq_msg(TWI_MSG_START, 0);
				_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_START, &msg, BUS_ACT, START_NO, W_INT);
				return;
			}

			// start pending
			if (p->start_pending) {
				avr_twi_msg_irq_t msg;
				msg = avr_twi_irq_msg(TWI_MSG_START, 0);
				_avr_twi_delay_state(p, AVR_TWI_START_CYCLES, TWI_START, &msg, BUS_ACT, START_NO, W_INT);
				return;
			}

			return;
		}

		// DATA, ACK or NACK received ?
		if (link->bus.msg == TWI_MSG_DATA
			|| link->bus.msg == TWI_MSG_CLK
			|| link->bus.msg == TWI_MSG_ACK
			|| link->bus.msg == TWI_MSG_NACK) {
			// ignore it, it is probably part of master-slave exchange
			return;
		}

		// slave active and waiting
		if (link->bus.msg == TWI_MSG_ADDR && _TWCR_COND_00x1) {
			avr_twi_msg_irq_t msg;

			// gen call ?
			if (link->bus.addr == 0x00 && avr->data[p->r_twar] & 0x01) {
				p->gencall = 1;
				msg = avr_twi_irq_msg(TWI_MSG_ACK, link->bus.addr);
				_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_GEN_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
				return;
			}

			// own address unset ?
			if ((avr->data[p->r_twar] >> 1) == 0x00) {
				msg = avr_twi_irq_msg(TWI_MSG_NACK, avr->data[p->r_twar]);	// send own address in response
				avr_raise_irq(p->io.irq + TWI_IRQ_OUTPUT, msg.v);
				_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_NO_STATE, NULL, BUS_UNCHG, START_UNCHG, WO_INT);
				return;
			}

			// slave own address ?
			if ((link->bus.addr >> 1) == (avr->data[p->r_twar] >> 1)) {
				msg = avr_twi_irq_msg(TWI_MSG_ACK, link->bus.addr);

				// SLA+R ?
				if (link->bus.addr & 0x01) {
					_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_STX_ADR_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
				} else {
				// SLA+W ?
					_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_ADR_ACK, &msg, BUS_UNCHG, START_UNCHG, W_INT);
				}
				return;
			}

			// not accessed or not responding to gencall
			msg = avr_twi_irq_msg(TWI_MSG_NACK, avr->data[p->r_twar]);	// send own address in response
			avr_raise_irq(p->io.irq + TWI_IRQ_OUTPUT, msg.v);
			_avr_twi_delay_state(p, AVR_TWI_NACK_CYCLES, TWI_NO_STATE, NULL, BUS_UNCHG, START_UNCHG, WO_INT);
			return;
		}

		// master active
		if (link->bus.msg == TWI_MSG_ADDR && _TWCR_COND_10x1) {
			avr_twi_msg_irq_t msg;

			// gen call ?
			if (link->bus.addr == 0x00 && avr->data[p->r_twar] & 0x01) {
				msg = avr_twi_irq_msg(TWI_MSG_ACK, link->bus.addr);
				_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_GEN_ACK_M_ARB_LOST, &msg, BUS_UNCHG, START_UNCHG, W_INT);
				return;
			}

			// slave own address ?
			if ((link->bus.addr >> 1) == (avr->data[p->r_twar] >> 1)) {
				msg = avr_twi_irq_msg(TWI_MSG_ACK, link->bus.addr);

				// SLA+R ?
				if (link->bus.addr & 0x01) {
					_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_STX_ADR_ACK_M_ARB_LOST, &msg, BUS_UNCHG, START_UNCHG, W_INT);
				} else {
				// SLA+W ?
					_avr_twi_delay_state(p, AVR_TWI_ACK_CYCLES, TWI_SRX_ADR_ACK_M_ARB_LOST, &msg, BUS_UNCHG, START_UNCHG, W_INT);
				}
				return;
			}

			// not accessed and arbitration lost
			msg = avr_twi_irq_msg(TWI_MSG_NULL, avr->data[p->r_twar]);	// send own address in response
			_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_ARB_LOST, &msg, BUS_UNCHG, START_UNCHG, W_INT);
			return;
		}
	}

	// unhandled case ==> go into error more
	_avr_twi_delay_state(p, AVR_TWI_NO_CYCLE, TWI_BUS_ERROR, NULL, BUS_INACT, START_NO, W_INT);
}

// elements match the TWI internal state (TWSR >> 3)
static void (*_avr_twi_fsm[])(struct avr_twi_t * p, struct twcr_t twcr, struct avr_twi_msg_irq_t * link) = {
	// TWI Miscellaneous
	_avr_twi_fsm_bus_error,					// 0x00

	// General TWI Master
	_avr_twi_fsm_start,						// 0x08
	_avr_twi_fsm_rep_start,					// 0x10

	// TWI Master Transmitter
	_avr_twi_fsm_mtx_adr_ack,				// 0x18
	_avr_twi_fsm_mtx_adr_nack,				// 0x20
	_avr_twi_fsm_mtx_data_ack,				// 0x28
	_avr_twi_fsm_mtx_data_nack,				// 0x30

	_avr_twi_fsm_arb_lost,					// 0x38

	// TWI Master Receiver
	_avr_twi_fsm_mrx_adr_ack,				// 0x40
	_avr_twi_fsm_mrx_adr_nack,				// 0x48
	_avr_twi_fsm_mrx_data_ack,				// 0x50
	_avr_twi_fsm_mrx_data_nack,				// 0x58

	// TWI Slave Receiver
	_avr_twi_fsm_srx_adr_ack,				// 0x60
	_avr_twi_fsm_srx_adr_ack_m_arb_lost,	// 0x68

	_avr_twi_fsm_srx_gen_ack,				// 0x70
	_avr_twi_fsm_srx_gen_ack_m_arb_lost,	// 0x78

	_avr_twi_fsm_srx_adr_data_ack,			// 0x80
	_avr_twi_fsm_srx_adr_data_nack,			// 0x88

	_avr_twi_fsm_srx_gen_data_ack,			// 0x90
	_avr_twi_fsm_srx_gen_data_nack,			// 0x98

	_avr_twi_fsm_srx_stop_restart,			// 0xa0

	// TWI Slave Transmitter
	_avr_twi_fsm_stx_adr_ack,				// 0xa8
	_avr_twi_fsm_stx_adr_nack,				// 0xb0
	_avr_twi_fsm_stx_data_ack,				// 0xb8
	_avr_twi_fsm_stx_data_nack,				// 0xc0
	_avr_twi_fsm_stx_ack_last_byte,			// 0xc8

	// filler for status code [0xd0 - 0xf0]
	_avr_twi_fsm_unknown,
	_avr_twi_fsm_unknown,
	_avr_twi_fsm_unknown,
	_avr_twi_fsm_unknown,
	_avr_twi_fsm_unknown,

	// TWI Miscellaneous
	_avr_twi_fsm_no_state,					// 0xf8
};


static void
avr_twi_write(
		struct avr_t * avr,
		avr_io_addr_t addr,
		uint8_t v,
		void * param)
{
	avr_twi_t * p = (avr_twi_t *)param;

	avr_core_watch_write(avr, addr, v);

	struct twcr_t twcr;
	twcr.twint = avr_regbit_get(avr, p->twi.raised);
	twcr.twea = avr_regbit_get(avr, p->twea);
	twcr.twsta = avr_regbit_get(avr, p->twsta);
	twcr.twsto = avr_regbit_get(avr, p->twsto);
	twcr.twen = avr_regbit_get(avr, p->twen);
	uint8_t twsr = avr_regbit_get_raw(p->io.avr, p->twsr);

	AVR_TWI_TRACE(avr, "TWI:[\x1b[94m%s\x1b[0m] %s 0x%02x\t(t=%d)\n",
			avr->tag_name, __func__, v, avr->cycle);
	AVR_TWI_TRACE(avr, "TWI:\t\tSTART:%d STOP:%d INT:%d EA:%d  bus:%d p:%d TWSR:%02x GC:%d --> ",
			twcr.twsta, twcr.twsto, twcr.twint, twcr.twea, p->bus_state, p->start_pending, twsr, p->gencall);


	if (twcr.twen) {
		(*_avr_twi_fsm[twsr >> 3])(p, twcr, NULL);
		AVR_TWI_TRACE(avr, "\n");
	} else {
		avr_regbit_setto_raw(p->io.avr, p->twsr, TWI_NO_STATE);
		avr_regbit_clear(avr, p->twea);
		avr_regbit_clear(avr, p->twsta);
		avr_regbit_clear(avr, p->twsto);
		avr_clear_interrupt(avr, &p->twi);
		AVR_TWI_TRACE(avr, "disabled\n");
	}

	if (twcr.twint)
		avr_clear_interrupt(avr, &p->twi);
}

/*
 * Write data to the latch, tell the system we have something
 * to send next
 */
static void
avr_twi_write_data(
		struct avr_t * avr,
		avr_io_addr_t addr,
		uint8_t v,
		void * param)
{
	avr_twi_t * p = (avr_twi_t *)param;

	AVR_TWI_TRACE(avr, "TWI:[\x1b[94m%s\x1b[0m] %s 0x%02x\t\t(t=%d)\n", avr->tag_name, __func__, v, avr->cycle);

	if (avr_regbit_get(avr, p->twi.raised)) {
		avr_core_watch_write(avr, addr, v);
		avr_regbit_set(avr, p->twwc);
	} else {
		avr_regbit_set(avr, p->twwc);
	}
}

/*
 * Read data from the latch, tell the system can receive a new byte
 */
static uint8_t
avr_twi_read_data(
		struct avr_t * avr,
		avr_io_addr_t addr,
		void * param)
{
	avr_twi_t * p = (avr_twi_t *)param;

	AVR_TWI_TRACE(avr, "TWI:[\x1b[94m%s\x1b[0m] %s 0x%02x\t\t(t=%d)\n", avr->tag_name, __func__, avr->data[p->r_twdr], avr->cycle);

	return avr->data[p->r_twdr];
}

/*
 * prevent code from rewriting out status bits, since we actually use them!
 */
static void
avr_twi_write_status(
		struct avr_t * avr,
		avr_io_addr_t addr,
		uint8_t v,
		void * param)
{
	avr_twi_t * p = (avr_twi_t *)param;
	uint8_t sr = avr_regbit_get(avr, p->twsr);

	avr_core_watch_write(avr, addr, v);
	avr_regbit_setto(avr, p->twsr, sr);	// force restore

	// if prescaler bits are changed, 
	// the new value will be taken into account 
	// only on the next posting of an hardware event
	// i.e. on call of _avr_twi_delay_state()
}

static void
avr_twi_irq_input(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	avr_twi_t * p = (avr_twi_t *)param;
	avr_t * avr = p->io.avr;

	struct twcr_t twcr;
	twcr.twint = avr_regbit_get(avr, p->twi.raised);
	twcr.twea = avr_regbit_get(avr, p->twea);
	twcr.twsta = avr_regbit_get(avr, p->twsta);
	twcr.twsto = avr_regbit_get(avr, p->twsto);
	twcr.twen = avr_regbit_get(avr, p->twen);
	uint8_t twsr = avr_regbit_get_raw(p->io.avr, p->twsr);

	avr_twi_msg_irq_t msg;
	msg.v = value;

	AVR_TWI_TRACE(avr, "TWI:\t[\x1b[94m%s\x1b[0m] %s msg %s  addr 0x%02x+%c / data 0x%02x\t(t=%d)\n", 
			avr->tag_name, __func__, msg2chr[msg.bus.msg], msg.bus.data >> 1, msg.bus.data & 0x01 ? 'R' : 'W', msg.bus.data, avr->cycle);

	AVR_TWI_TRACE(avr, "TWI:\t\tSTART:%d STOP:%d INT:%d EA:%d  bus:%d p:%d TWSR:%02x GC:%d --> ",
			twcr.twsta, twcr.twsto, twcr.twint, twcr.twea, p->bus_state, p->start_pending, twsr, p->gencall);

	(*_avr_twi_fsm[twsr >> 3])(p, twcr, &msg);
}

void avr_twi_reset(struct avr_io_t *io)
{
	avr_twi_t * p = (avr_twi_t *)io;
	avr_irq_register_notify(p->io.irq + TWI_IRQ_INPUT, avr_twi_irq_input, p);

	// set reset value
	avr_regbit_setto_raw(p->io.avr, p->twsr, TWI_NO_STATE);
}

static const char * irq_names[TWI_IRQ_COUNT] = {
	[TWI_IRQ_INPUT] = "8<input",
	[TWI_IRQ_OUTPUT] = "32>output",
	[TWI_IRQ_STATUS] = "8>status",
};

static	avr_io_t	_io = {
	.kind = "twi",
	.reset = avr_twi_reset,
	.irq_names = irq_names,
};

void avr_twi_init(avr_t * avr, avr_twi_t * p)
{
	p->io = _io;
	avr_register_io(avr, &p->io);
	avr_register_vector(avr, &p->twi);

	// allocate this module's IRQ
	avr_io_setirqs(&p->io, AVR_IOCTL_TWI_GETIRQ(p->name), TWI_IRQ_COUNT, NULL);

	avr_register_io_write(avr, p->twen.reg, avr_twi_write, p);
	avr_register_io_write(avr, p->r_twdr, avr_twi_write_data, p);
	avr_register_io_read(avr, p->r_twdr, avr_twi_read_data, p);
	avr_register_io_write(avr, p->twsr.reg, avr_twi_write_status, p);
}

struct avr_twi_msg_irq_t
avr_twi_irq_msg(
		avr_twi_msg_t msg,
		uint8_t addr_data)
{
	struct avr_twi_msg_irq_t _msg;

	_msg.bus.msg = msg;
	_msg.bus.addr = addr_data;

	return _msg;
}

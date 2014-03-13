/*
	avr_twi.h

	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>

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

#ifndef __AVR_TWI_H__
#define __AVR_TWI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sim_avr.h"

//#include "sim_twi.h"

enum {
	TWI_IRQ_INPUT = 0,
	TWI_IRQ_OUTPUT,
	TWI_IRQ_STATUS,
	TWI_IRQ_COUNT
};

typedef enum avr_twi_msg_t {
	TWI_MSG_NULL,
	TWI_MSG_START,
	TWI_MSG_ADDR,
	TWI_MSG_DATA,
	TWI_MSG_ACK,
	TWI_MSG_NACK,
	TWI_MSG_CLK,
	TWI_MSG_STOP,
	TWI_MSG_CNT,
} avr_twi_msg_t;

typedef struct avr_twi_bus_msg_t {
	enum avr_twi_msg_t msg : 8;
    union {
        uint16_t addr : 8;
        uint16_t data : 8;
    };
} avr_twi_bus_msg_t;

typedef struct avr_twi_msg_irq_t {
	union {
		uint32_t v;
		avr_twi_bus_msg_t bus;
	};
} avr_twi_msg_irq_t;

// add port number to get the real IRQ
#define AVR_IOCTL_TWI_GETIRQ(_name) AVR_IOCTL_DEF('t','w','i',(_name))

typedef struct avr_twi_t {
	avr_io_t	io;
	char name;

	avr_regbit_t	disabled;	// bit in the PRR

	avr_io_addr_t	r_twbr;			// bit rate register
	avr_io_addr_t	r_twcr;			// control register
	avr_io_addr_t	r_twsr;			// status register
	avr_io_addr_t	r_twar;			// address register (slave)
	avr_io_addr_t	r_twamr;		// address mask register
	avr_io_addr_t	r_twdr;			// data register
	
	avr_regbit_t twen;		// twi enable bit
	avr_regbit_t twea;		// enable acknowledge bit
	avr_regbit_t twsta;		// start condition
	avr_regbit_t twsto;		// stop condition
	avr_regbit_t twwc;		// write collision
	
	avr_regbit_t twsr;		// status registers, (5 bits)
	avr_regbit_t twps;		// prescaler bits (2 bits)
	
	avr_int_vector_t twi;	// twi interrupt

	uint8_t bus_state:1;	// 1 if bus is active
	uint8_t start_pending:1;	// start requested but bus is already active

	uint8_t gencall:1;	// general call handling flag
} avr_twi_t;

void
avr_twi_init(
		avr_t * avr,
		avr_twi_t * port);

/*
 * Create a message value for twi including the 'msg' bitfield,
 * 'addr' and data. This value is what is sent as the IRQ value
 */
struct avr_twi_msg_irq_t
avr_twi_irq_msg(
		avr_twi_msg_t msg,
		uint8_t addr_data);

#ifdef __cplusplus
};
#endif

#endif /*__AVR_TWI_H__*/

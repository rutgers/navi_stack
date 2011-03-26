/*
 * encoder_setup.h
 *
 *  Created on: Mar 16, 2011
 *      Author: asher
 */

#ifndef ENCODER_SETUP_H_
#define ENCODER_SETUP_H_

#include <WProgram.h>
#include <avr/interrupt.h>

/*
 * All of our Encoders inputs using the pin change interrupts
 * on PORTC.
 * *
	ENC_IN(PC2, PC3)  // PCINT10, PCINT11
 *  Encoder 1
 *  A2  (PC2) -> M1.ENA
	A3 ((PC3)-> M1.ENB


 * 	ENC_IN(PC4, PC5), // PCINT12, PCINT13
	Encoder 2
	A4 (PC4)-> M2.ENA
	A5 (PC5)-> M2.ENB
 */


#define enc_pin_init(DDR_REG, PIN, ENC_PCMASK) do {					\
	/* set pin as input and unmask in pcint register */	\
	DDR_REG &=     ~_BV(PIN);						\
	ENC_PCMASK     |=  _BV(PIN);						\
} while(0)



static void enc_init(void)
{
	cli();
	PCMSK1 =0;
	enc_pin_init(DDRC, PC2, PCMSK1); // PCINT10, PCINT11
	enc_pin_init(DDRC, PC3, PCMSK1);

	enc_pin_init(DDRC, PC4, PCMSK1); // PCINT12, PCINT13
	enc_pin_init(DDRC, PC5, PCMSK1);

	PORTC |= _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5);

	PCICR = _BV(PCIE1);
	sei();
}



#define enc_isr_off() do {		\
	PCICR &= ~(1 << ENC_PCIE);	\
	barrier();			\
} while(0)

#define enc_isr_on() do {		\
	PCICR |= (1 << ENC_PCIE);	\
	barrier();			\
} while(0)



#endif /* ENCODER_SETUP_H_ */

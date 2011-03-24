/*
 * pd_setup.h
 *
 *  Created on: Mar 17, 2011
 *      Author: asher
 */

#ifndef PD_SETUP_H_
#define PD_SETUP_H_
#include <avr/interrupt.h>

void setup_control_loop(){
	cli();
		 // Choose system as clock source with 1/64 prescaler
	TCCR3B = 0b00000011;
	TCNT3 = 0xFFFF - 12500;      // Compare time set to 50 millisecs by setting TCNT3 to 64563-40,000
	TIMSK3 = _BV(TOIE3);  // Set interrupt on compare match

	sei();
}

#endif /* PD_SETUP_H_ */

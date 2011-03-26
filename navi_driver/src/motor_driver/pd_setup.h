/*
 * pd_setup.h
 *
 *  Created on: Mar 17, 2011
 *      Author: asher
 */

#ifndef PD_SETUP_H_
#define PD_SETUP_H_
#include <avr/interrupt.h>

void initPIDTimer(){
	cli();

	//This timer sets the clock to overflow
	//The period is 0.016384 seconds.
	//This the interrupt occurs 61 a second

		 // Choose system as clock source with 1024 prescaler
	TCCR2B = 0b00000101;
	TCNT2 = 0;      // Compare time set to 20 millisecs by setting TCNT3 to 64563-40,000
	TIMSK2 = _BV(TOIE2);  // Set interrupt on compare match
	sei();
}



#endif /* PD_SETUP_H_ */

#include <stdint.h>
#include <util/delay.h>
#include "pid.hpp"

volatile uint16_t pid_count = 0;

void pid_init(void)
{
	// Reset the count to zero (TCNT2), interrupt on match (TIMSK2), and set
	// the target value (OCR2A). Use a prescaler of 1024 (i.e. 3'b111).
	TCCR2B = 7;
	TCNT2  = 0;
	TIMSK2 = 1 << OCIE2A;
	TIFR2  = 1 << OCF2A;
	OCR2A  = 255;
}

ISR(TIMER2_OVF_vect)
{
	TCNT2 = 0;

}

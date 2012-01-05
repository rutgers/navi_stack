#include <WProgram.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "config.hpp"
#include "encoder.hpp"

static volatile uint8_t last_enc1a = 0;
static volatile uint8_t last_enc1b = 0;
static volatile uint8_t last_enc2a = 0;
static volatile uint8_t last_enc2b = 0;

volatile int16_t motor1_ticks = 0;
volatile int16_t motor2_ticks = 0;

void encoder_init(void)
{
	// Set pins as inputs (DDRD = 0) with an internal pull-up (PORTD = 1).
	DDRD  &= ~(1 << PIN2 | 1 << PIN3
	         | 1 << PIN4 | 1 << PIN5);
	PORTD |=   1 << PIN2 | 1 << PIN3
	         | 1 << PIN4 | 1 << PIN5;

	// Enable pin-change interrupts.
	cli();
	PCICR |= 1 << PCIE2;
	PCMSK2 = 1 << PCINT18 | 1 << PCINT19 | 1 << PCINT20 | 1 << PCINT21;
	sei();
}

static inline int8_t encoder_tick(uint8_t last_a, uint8_t last_b,
                                  uint8_t curr_a, uint8_t curr_b,
								  volatile int16_t *ticks)
{
	if (!last_a && curr_a) {
		if (!curr_b)
			*ticks -= 1;
		else
			*ticks += 1;
	} else if (last_a && !curr_a) {
		if (!curr_b)
			*ticks += 1;
		else
			*ticks -= 1;
	}

	if (!last_b && curr_b) {
		if (!curr_a)
			*ticks += 1;
		else
			*ticks -= 1;
	} else if (last_b && !curr_b) {
		if (!curr_a)
			*ticks -= 1;
		else
			*ticks += 1;
	}
}

ISR(PCINT2_vect)
{
	uint8_t const enc1a = !!(PIND & 1 << PIN2);
	uint8_t const enc1b = !!(PIND & 1 << PIN3);
	uint8_t const enc2a = !!(PIND & 1 << PIN4);
	uint8_t const enc2b = !!(PIND & 1 << PIN5);

	encoder_tick(last_enc1a, last_enc1b, enc1a, enc1b, &motor1_ticks);
	encoder_tick(last_enc2a, last_enc2b, enc2a, enc2b, &motor2_ticks);

	last_enc1a = enc1a;
	last_enc1b = enc1b;
	last_enc2a = enc2a;
	last_enc2b = enc2b;
}

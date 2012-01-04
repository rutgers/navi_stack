#include <stdbool.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "config.hpp"
#include "encoder.hpp"

static volatile bool last_enc1a = 0;
static volatile bool last_enc1b = 0;
static volatile bool last_enc2a = 0;
static volatile bool last_enc2b = 0;

volatile int16_t motor1_ticks = 0;
volatile int16_t motor2_ticks = 0;

void encoder_init(void)
{
	// Set pins as inputs (DDRD = 0) with an internal pull-up (PORTD = 1).
	DDRD  &= ~(1 << MOTOR1_ENCA | 1 << MOTOR1_ENCB
	         | 1 << MOTOR2_ENCA | 1 << MOTOR2_ENCB);
	PORTD |=   1 << MOTOR1_ENCA | 1 << MOTOR1_ENCB
	         | 1 << MOTOR1_ENCB | 1 << MOTOR2_ENCB;

	// Enable pin-change interrupts.
	cli();
	PCICR |= 1 << PCIE2;
	PCMSK2 = 1 << PCINT18 | 1 << PCINT19 | 1 << PCINT20 | 1 << PCINT21;
	sei();
}

static inline int8_t encoder_tick(bool last_a, bool last_b,
                                  bool curr_a, bool curr_b)
{
	if (!last_a && curr_a) {
		return (!curr_b) ? -1 : +1;
	} else if (last_a && !curr_a) {
		return (!curr_b) ? +1 : -1;
	}
}

ISR(PCINT2_vect)
{
	bool const enc1a = PIND | 1 << MOTOR1_ENCA;
	bool const enc1b = PIND | 1 << MOTOR1_ENCB;
	bool const enc2a = PIND | 1 << MOTOR2_ENCA;
	bool const enc2b = PIND | 1 << MOTOR2_ENCB;

	motor1_ticks += encoder_tick(last_enc1a, last_enc1b, enc1a, enc1b);
	motor2_ticks += encoder_tick(last_enc2a, last_enc2b, enc2a, enc2b);

	last_enc1a = enc1a;
	last_enc1b = enc1b;
	last_enc2a = enc2a;
	last_enc2b = enc2b;
}

#include <WProgram.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "config.hpp"
#include "encoder.hpp"

encoder_t encoders[ENCODERS_NUM] = {
	{ MOTOR1_ENCA, MOTOR1_ENCB, 0, 0, 0, 0 },
	{ MOTOR2_ENCA, MOTOR2_ENCB, 0, 0, 0, 0 }
};

void encoder_init(void)
{
	// Set pins as inputs (DDRD = 0) with an internal pull-up (PORTD = 1).
	// TODO: Generalize the initialization using the array of encoder structs.
	DDRD  &= ~(1 << PIN2 | 1 << PIN3 | 1 << PIN4 | 1 << PIN5);
	PORTD |=   1 << PIN2 | 1 << PIN3 | 1 << PIN4 | 1 << PIN5;

	cli();
	PCICR |= _BV(PCIE2);
	PCMSK2 = _BV(PCINT18) | _BV(PCINT19) | _BV(PCINT20) | _BV(PCINT21);
	sei();
}

static inline int8_t encoder_tick(encoder_t *const encoder)
{
	bool const last_a = encoder->pin1_last;
	bool const last_b = encoder->pin2_last;
	bool const curr_a = digitalRead(encoder->pin1);
	bool const curr_b = digitalRead(encoder->pin2);
	int8_t delta = 0;

	if (!last_a && curr_a) {
		if (!curr_b)
			delta--;
		else
			delta++;
	} else if (last_a && !curr_a) {
		if (!curr_b)
			delta++;
		else
			delta--;
	}

	if (!last_b && curr_b) {
		if (!curr_a)
			delta++;
		else
			delta--;
	} else if (last_b && !curr_b) {
		if (!curr_a)
			delta++;
		else
			delta--;
	}

	encoder->ticks_short += delta;
	encoder->ticks_long  += delta;
	encoder->pin1_last = curr_a;
	encoder->pin2_last = curr_b;
}

ISR(PCINT2_vect)
{
	for (size_t i = 0; i < ENCODERS_NUM; i++) {
		encoder_tick(&encoders[i]);
	}
}

#include <stdint.h>
#include <WProgram.h>
#include <util/atomic.h>
#include "encoder.hpp"
#include "pid.hpp"

volatile uint16_t pid_count = 0;
static int16_t angvel_setpt = 0;

// Previous velocity used to estimate the derivative of error.
static int16_t last_angvel1 = 0;
static int16_t last_angvel2 = 0;

// Counter used to estimate the integral of error.
static int16_t int_angvel1 = 0;
static int16_t int_angvel2 = 0;

// PID Parameters.
static int16_t int_max = 255;
static float kp1 = 0.0f, kp2 = kp1;
static float ki1 = 0.0f, ki2 = ki1;
static float kd1 = 0.0f, kd2 = kd1;

#define CONSTRAIN(_x_, _a_, _b_) \
    (((_x_) < (_a_)) ? (_a_) :   \
    (((_x_) > (_b_)) ? (_b_) :   \
                       (_x_)));

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

void pid_set_angvel(int16_t ticks_per_loop)
{
	angvel_setpt = ticks_per_loop;
}

ISR(TIMER2_COMPA_vect)
{
	int16_t angvel1, angvel2;
	TCNT2 = 0;

	// Reset encoder ticks to prevent overflow. This must be an atomic
	// operation because 16-bit operations require multiple instructions.
	ATOMIC_BLOCK (ATOMIC_FORCEON) {
		angvel1 = motor1_ticks;
		angvel2 = motor2_ticks;
		motor1_ticks = 0;
		motor2_ticks = 0;
	}

	int16_t const error = angvel_setpt - angvel1;
	int16_t const prop1 = error;
	int16_t const diff1 = angvel1 - last_angvel1;
	int16_t const int1  = CONSTRAIN(int1 + error, -int_max, int_max);

	// Calculate the PWM output using PID.
	float   const pwm_raw = (kp1 * prop1) + (kd1 * diff1) + (ki1 * int1);
	int16_t const pwm     = (int16_t)CONSTRAIN(pwm_raw, -255.0, 255.0);

	// TODO: Set motor outputs.
	pid_count++;
}

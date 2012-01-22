#include <math.h>
#include <stdint.h>
#include <WProgram.h>
#include <util/atomic.h>
#include "config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "pid.hpp"

#define CONSTRAIN(_x_, _a_, _b_) (((_x_) < (_a_)) ? (_a_) :         \
                                 (((_x_) > (_b_)) ? (_b_) : (_x_)))
#define THRESHOLD(_x_, _t_) ((-(_t_) <= (_x_) && (_x_) <= +(_t_)) ? (_x_) : 0)


pid_t pids[PIDS_NUM] = { 0 };
bool pid_enable = false;
volatile uint16_t pid_ticks = 0;

void pid_init(void)
{
	// Interrupt on match (TIMSK2) and set the target value (OCR2A).
	TIMSK2 = 1 << OCIE2A;
	TIFR2  = 1 << OCF2A;
	OCR2A  = 255;

	// Set the prescaler to 1024 and calculate the duration of one period.
	TCCR2B = 0x0007; // = 00000111 = 1024

	// Reset the timer.
	TCNT2  = 0;
}

static int16_t pid_tick(pid_t *pid, int16_t value)
{
	int16_t const error      = pid->target - value;
	int16_t const derivative = value - pid->last;
	int16_t const integral   = CONSTRAIN(pid->integral + error,
	                                     -pid->integral_max, pid->integral_max);

	float const pwm_raw = pid->kf * pid->feedforward
	                    + pid->kp * error
	                    + pid->ki * integral
	                    + pid->kd * derivative;

	// Prevent integral windup when the controller is saturated and by capping
	// the integral at a reasonable value.
	if (abs(pwm_raw) < PWM_MAX && abs(pid->integral) < pid->integral_max) {
		pid->integral += error;
	}
	pid->last = value;

	return (int16_t)CONSTRAIN(pwm_raw, -PWM_MAX, PWM_MAX);
}

ISR(TIMER2_COMPA_vect)
{
	int16_t ticks[PIDS_NUM], pwms[PIDS_NUM];

#if 0
	// Accumulate the encoder ticks in a buffer for debugging. We need to be
	// especially careful here because interrupts are still enabled and 16-bit
	// operations are not atomic.
	ATOMIC_BLOCK (ATOMIC_FORCEON) {
		for (size_t i = 0; i < ENCODERS_NUM; i++) {
			ticks[i] = encoders[i].ticks_short;
			encoders[i].ticks_short = 0;
		}
	}

	// TODO: Generalize this to an arbitrary number of motors.
	for (size_t i = 0; i < ENCODERS_NUM; i++) {
		if (pids[i].enable) {
			pwms[i] = pid_tick(&pids[i], ticks[i]);
		}
	}
	//motor_set(pwms[0], pwms[1]);
#endif

	// Reset the timer.
	pid_ticks++;
	TCNT2 = 0;
}

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

#define US(_s_) (1000000ul * (_s_))
#define PERIOD_US(_ticks_, _pres_) ((uint16_t)((US(1) * (_pres_) / F_CPU) * (_ticks_)))

#define TIMER_MAX 255

uint16_t period_us = 0;

struct pid_t {
	int16_t target;
	int16_t threshold;
	int16_t last;
	int16_t integral;
	int16_t integral_max;
	float kp, ki, kd;
};

static struct pid_t pid1 = { 0 };
static struct pid_t pid2 = { 0 };

void pid_init(void)
{
	// Interrupt on match (TIMSK2) and set the target value (OCR2A).
	TIMSK2 = 1 << OCIE2A;
	TIFR2  = 1 << OCF2A;
	OCR2A  = TIMER_MAX;

	// Set the prescaler to 1024 and calculate the duration of one period.
	TCCR2B = 0x0007; // = 00000111 = 1024
	period_us = PERIOD_US(TIMER_MAX, 1024); // ~= 60 Hz

	// Reset the timer.
	TCNT2  = 0;

	pid1.integral     = pid2.integral     = 0;
	pid1.integral_max = pid2.integral_max = 255;
	pid1.kp = pid2.kp = 0.0f;
	pid1.ki = pid2.ki = 0.0f;
	pid1.kd = pid2.kd = 0.0f;
}

void pid_set_target(float omega1, float omega2)
{
	// Note that omega has units of rad/s and period_us has units of us.
	//   ticks/sec = (ticks/rev) * (rev/sec) * (sec)
	pid1.target = (int16_t)(TICKS_PER_REV * omega1 * period_us) / (2*PI * US(1));
	pid2.target = (int16_t)(TICKS_PER_REV * omega2 * period_us) / (2*PI * US(1));
}

static int16_t pid_tick(struct pid_t *pid, int16_t value)
{
	int16_t const error = THRESHOLD(pid->target - value, pid->threshold);
	int16_t const prop = error;
	int16_t const diff = value - pid1.last;
	int16_t const inte = CONSTRAIN(pid->integral + error, -pid->integral_max,
	                               pid->integral_max);

	// Calculate the PWM output using PID.
	float const pwm_raw = pid->kp * prop + pid->kd * diff + pid->ki * inte;
	return (int16_t)CONSTRAIN(pwm_raw, -255.0, 255.0);

}

ISR(TIMER2_COMPA_vect)
{
#if 0
	int16_t const pwm1 = pid_tick(&pid1, motor1_ticks);
	int16_t const pwm2 = pid_tick(&pid2, motor2_ticks);
	motor_set(pwm1, pwm2);

	motor1_ticks = 0;
	motor2_ticks = 0;
	TCNT2 = 0;
#endif
}

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

struct pid_t pid1 = { 0 };
struct pid_t pid2 = { 0 };
volatile int16_t encoder1_buffer = 0;
volatile int16_t encoder2_buffer = 0;

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
	int16_t const pwm1 = pid_tick(&pid1, motor1_ticks);
	int16_t const pwm2 = pid_tick(&pid2, motor2_ticks);
	//motor_set(pwm1, pwm2);

	// Accumulate the encoder ticks in a buffer for debugging.
	encoder1_buffer += motor1_ticks;
	encoder2_buffer += motor2_ticks;
	motor1_ticks = 0;
	motor2_ticks = 0;
	TCNT2 = 0;
}

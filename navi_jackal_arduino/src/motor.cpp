#include <stdint.h>
#include <WProgram.h>
#include "config.hpp"
#include "motor.hpp"

void motor_set_one(int16_t vel, int16_t pin_dir, int16_t pin_pwm)
{
	if (vel > 0) {
		digitalWrite(pin_dir, 0);
		analogWrite(pin_pwm, vel);
	} else {
		digitalWrite(pin_dir, 1);
		analogWrite(pin_pwm, vel + 255);
	}
}

void motor_init(void)
{
	pinMode(MOTOR1_DIR, OUTPUT);
	pinMode(MOTOR2_DIR, OUTPUT);
	pinMode(MOTOR1_EN, OUTPUT);
	pinMode(MOTOR2_EN, OUTPUT);

	motor_set(0, 0);
	motor_enable(true);
}

void motor_enable(bool enabled)
{
	digitalWrite(MOTOR1_EN, enabled);
	digitalWrite(MOTOR2_EN, enabled);
}

void motor_set(int16_t left, int16_t right)
{
	motor_set_one(left, MOTOR1_DIR, MOTOR1_PWM);
	motor_set_one(right, MOTOR2_DIR, MOTOR2_PWM);
}

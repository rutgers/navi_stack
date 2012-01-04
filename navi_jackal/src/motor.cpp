#include <stdint.h>
#include <WProgram.h>
#include "config.hpp"
#include "motor.hpp"

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
	digitalWrite(MOTOR1_DIR, left < 0);
	digitalWrite(MOTOR2_DIR, right < 0);
	analogWrite(MOTOR1_PWM, (uint8_t)abs(left));
	analogWrite(MOTOR2_PWM, (uint8_t)abs(right));
}

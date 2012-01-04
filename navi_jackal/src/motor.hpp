#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include <stdint.h>

void motor_init(void);
void motor_enable(bool enabled);
void motor_set(int16_t left, int16_t right);

#endif

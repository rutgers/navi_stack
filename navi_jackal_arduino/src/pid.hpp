#ifndef PID_HPP_
#define PID_HPP_

#include <stdint.h>

extern volatile int16_t encoder1_buffer;
extern volatile int16_t encoder2_buffer;

extern uint16_t period_us;

void pid_init(void);
void pid_set_constants(float kf, float kp, float kd, float ki, int16_t threshold);

#endif

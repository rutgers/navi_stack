#ifndef PID_HPP_
#define PID_HPP_

#include <stdint.h>

extern volatile uint16_t pid_count;
extern uint16_t period_us;

void pid_init(void);
void pid_set_angvel(float omega);

#endif

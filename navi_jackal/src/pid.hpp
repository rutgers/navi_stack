#ifndef PID_HPP_
#define PID_HPP_

#include <stdint.h>

extern volatile uint16_t pid_count;

void pid_init(void);
void pid_set_angvel(int16_t ticks_per_loop);

#endif

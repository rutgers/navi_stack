#ifndef PID_HPP_
#define PID_HPP_

#include <stdint.h>

extern volatile uint16_t pid_count;

void pid_init(void);

#endif

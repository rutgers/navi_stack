#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include <stdint.h>

extern volatile int16_t motor1_ticks;
extern volatile int16_t motor2_ticks;

void encoder_init(void);

#endif

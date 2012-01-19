#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include <stdint.h>

#define ENCODERS_NUM 2
#define ENCODERS_PERIOD_MS 25

struct encoder_t {
	bool const pin1, pin2;
	volatile bool pin1_last, pin2_last;
	volatile int16_t ticks_short;
	volatile int32_t ticks_long;
};

extern encoder_t encoders[ENCODERS_NUM];

void encoder_init(void);

#endif

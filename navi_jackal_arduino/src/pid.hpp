#ifndef PID_HPP_
#define PID_HPP_

#include <stdint.h>

struct pid_t {
	int16_t target;
	int16_t feedforward;
	int16_t threshold;
	int16_t last;
	int16_t integral;
	int16_t integral_max;
	float kf, kp, ki, kd;
};

extern pid_t pid1, pid2;
extern volatile int16_t encoder1_buffer, encoder2_buffer;

void pid_init(void);

#endif

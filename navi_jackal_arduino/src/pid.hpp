#ifndef PID_HPP_
#define PID_HPP_

#include <stdint.h>

#define PIDS_NUM 2

struct pid_t {
	bool enable;
	int16_t target;
	int16_t feedforward;
	int16_t threshold;
	int16_t last;
	int16_t integral;
	int16_t integral_max;
	float kf, kp, ki, kd;
};

extern pid_t pids[PIDS_NUM];
extern bool pid_enable;
extern volatile uint16_t pid_ticks;

void pid_init(void);

#endif

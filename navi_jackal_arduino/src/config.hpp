#ifndef CONFIG_HPP_
#define CONFIG_HPP_

// Enable the calibration service for measuring the motor curve.
#define JACKAL_CALIBRATION

#define MOTOR1_DIR 7
#define MOTOR2_DIR 8
#define MOTOR1_PWM 9
#define MOTOR2_PWM 10
#define MOTOR1_EN  11
#define MOTOR2_EN  12

#define MOTOR1_ENCA 2
#define MOTOR1_ENCB 3
#define MOTOR2_ENCA 4
#define MOTOR2_ENCB 5

#define PWM_MAX 255
#define TICKS_PER_REV 1000

#endif

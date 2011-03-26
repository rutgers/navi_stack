/*
 * PDMotorController.h
 *
 *  Created on: Mar 9, 2011
 *      Author: asher
 */

#ifndef PDMotorController_H_
#define PDMotorController_H_
#include "Encoder.h"
#include <avr/interrupt.h>
#include "WProgram.h"


class PDMotorController {
public:
	// arduino pwmPin && direciton pin
	// PID constants K and D
	// what is the PID update frequency

	PDMotorController(int16_t p_k, int16_t p_d,
			int16_t pwm_max);
	~PDMotorController(){};
	//encoder counts per second
	int16_t getVelocity();

	//motor cmd output from PD controller
	int16_t motorCMD();

	//set the set point velocity for the PD controller
	void setVelocity(int16_t encoders_per_second);

	//Must be called during an timer interrupt
	void PIDUpdate(); //compute closed loop update
					  // updates the motor control signal to reach the speed
	void encoderUpdate(char channelA, char channelB){
		encoder.update(channelA,channelB);
	}
	int encoderCount(){return encoder.count();};
	void clearEncoder(){
		cli();
		encoder.clearCount(); this->prior_encoder_count=0;
		sei();}

	int pwm;

private:
	Encoder encoder;
	int16_t update_freqency;
	int16_t prior_encoder_count;
	int16_t enc_vel;
	int16_t commanded_vel;
	int16_t pwm_max;
	int16_t error_p;
	int16_t p_k;
	int16_t p_d;
};

#endif /* PDMotorController_H_ */

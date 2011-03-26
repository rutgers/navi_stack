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


class PDMotorController {
public:
	// arduino pwmPin && direciton pin
	// PID constants K and D
	// what is the PID update frequency

	PDMotorController(int p_k, int p_d,
					int pwm_max, int update_frequency);
	~PDMotorController(){};
	//encoder counts per second
	int getVelocity();

	//motor cmd output from PD controller
	int motorCMD();

	//set the set point velocity for the PD controller
	void setVelocity(int encoders_per_second);

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
	int update_freqency;
	int prior_encoder_count;
	int enc_vel;
	int commanded_vel;
	int pwm_max;
	int error_p;
	int p_k;
	int p_d;
};

#endif /* PDMotorController_H_ */

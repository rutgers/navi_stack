/*
 * PDMotorController.cpp
 *
 *  Created on: Mar 9, 2011
 *      Author: asher
 */

#include "PDMotorController.h"
#include "WProgram.h"

PDMotorController::PDMotorController(int16_t p_k, int16_t p_d, int16_t pwm_max){
	this->p_k = p_k;
	this->p_d = p_d;
	this->pwm_max = pwm_max;
	this->pwm = 0;
	this->enc_vel=0;
}

int16_t PDMotorController::getVelocity(){
	return this->enc_vel;
}

int16_t PDMotorController::motorCMD(){
	return pwm;
}

void PDMotorController::setVelocity(int commanded_vel){
	this->commanded_vel = commanded_vel;
}

void PDMotorController::PIDUpdate(){
	this->enc_vel = encoder.count() - this->prior_encoder_count;
	this->prior_encoder_count = encoder.count();

	//compute error amount
	int error = this->enc_vel-this->commanded_vel;

	this->pwm =this->pwm -  p_k* error ;//p_d* (error-error_p);
	this->error_p = error;

	if (pwm >= pwm_max) pwm = pwm_max;
	if (pwm < (-pwm_max)) pwm = -pwm_max;
}



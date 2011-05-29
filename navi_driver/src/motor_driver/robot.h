/*
 * robot.h
 *
 *  Created on: Dec 9, 2009
 *      Author: asher
 */

#ifndef ROBOT_H_
#define ROBOT_H_



#define feedBackLoopFrequency 30.5

#include "Encoder.h"


void RobotInit();

struct wheel
		{
			int kpGain;  // proportional gain constant
			int kiGain; //  integral gain constant
			int kDGain; // derivative gain constant
			Encoder encoder;
			int previousCount;
			int vel;
			int velD;
			int countD; // desired distance in encoder counts
			int velCM;   // current velocity counts
			int error;
			int errorp;
			int errorT;
			char blacklashFlag;// Use for backlash control
			int pwmPeriod;

		};

struct robotState{
	  wheel rightWheel,  leftWheel;
		char feedbackState;
		char clearEncoder;
};

extern robotState Robot;


 void initEncoders();
 void initSpeedControl();

 void setLeftMotorSpeed(int cmPerSec);
 void setRightMotorSpeed(int cmPerSec);
 void move(float cmPerSec, float distanceCM);
 void turn(float cmPerSec, int AngleDegrees);



void RobotInit();



#endif /* ROBOT_H_ */

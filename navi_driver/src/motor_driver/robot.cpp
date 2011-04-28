/*
Robot.c


*/

#include "robot.h"
#include "WProgram.h"
#include <stdint.h>
#include "motorcontrol.h"


robotState Robot;


void RobotInit(void){

	initMotorDriver();
	initEncoders();
	initSpeedControl();
}

void initEncoders()
{
	//************* Intialize encoders	*********************
	DDRD &=  ~_BV(PIN2) & ~_BV(PIN3)& ~_BV(PIN4)& ~_BV(PIN5);
	PORTD = _BV(PIN2) | _BV(PIN3) |  _BV(PIN4) | _BV(PIN5);

	cli();

	/*
	 * Using PCINT18,19,20,6
	 */
	PCICR  |= _BV(PCIE2);
	PCMSK2 = _BV(PCINT18) | _BV(PCINT19) | _BV(PCINT20) | _BV(PCINT21);

	sei();

}


ISR(PCINT2_vect){
	Robot.leftWheel.encoder.update(bit_is_set(PIND, PIN5), bit_is_set(PIND, PIN4));
	Robot.rightWheel.encoder.update( bit_is_set(PIND, PIN2), bit_is_set(PIND, PIN3));
}


void initSpeedControl()
{
	// this function initializes the timer for the feed back
	// loop controlling the speed

	cli();

	//Feed back is going to be set for a prescaller of 1024 on a 8 bit timer, feed back loop is
	//  32.8 miliseoncs

	TCCR2B = 0b00000111;
	TCNT2 = 0;
	TIMSK2 = _BV(TOIE2);  // Set interrupt on compare match

	Robot.rightWheel.vel=0;
	Robot.rightWheel.velD = 0;
	Robot.rightWheel.encoder.clearCount();
	Robot.rightWheel.countD = 0;
	Robot.rightWheel.errorp =0;
	Robot.rightWheel.kiGain =0;

	Robot.leftWheel.vel = 0;
	Robot.leftWheel.velD = 0;
	Robot.leftWheel.encoder.clearCount();
	Robot.leftWheel.countD = 0;
	Robot.leftWheel.errorp=0;
	Robot.leftWheel.kiGain =0;

	Robot.feedbackState = 0;
	sei();
}


int limitSpeedMax(int speed)
{
	if (speed > 255) return 255;  // max duty cycle is 510
	if (speed < -255) return -255; // max duty cycle is 510
	return speed;
}





ISR(TIMER2_OVF_vect)   // feed back loop interrupt
{
	//TCNT2 = 0;      // Feedbackloop modifies every 32.8 miliseconds

	if (Robot.feedbackState == 1){

		Robot.rightWheel.vel = Robot.rightWheel.encoder.count() - Robot.rightWheel.previousCount;

		Robot.leftWheel.vel = Robot.leftWheel.encoder.count() - Robot.leftWheel.previousCount;
		if (Robot.clearEncoder){
			Robot.rightWheel.previousCount = 0;
			Robot.leftWheel.previousCount = 0;
			Robot.rightWheel.encoder.clearCount();
			Robot.leftWheel.encoder.clearCount();
			Robot.clearEncoder = 0;
		}


	Robot.rightWheel.error = Robot.rightWheel.velD - Robot.rightWheel.vel;
	Robot.leftWheel.error = Robot.leftWheel.velD - Robot.leftWheel.vel;


	Robot.rightWheel.kpGain = Robot.rightWheel.error*3/4;
	Robot.leftWheel.kpGain = Robot.leftWheel.error*3/4;
	
	
	Robot.rightWheel.kDGain = (Robot.rightWheel.errorp -Robot.rightWheel.error)*3/4;
	Robot.leftWheel.kDGain = (Robot.leftWheel.errorp -Robot.leftWheel.error)*3/4;

	Robot.rightWheel.kiGain = 0;
	Robot.leftWheel.kiGain = 0;


	Robot.rightWheel.errorp = Robot.rightWheel.error;
	Robot.leftWheel.errorp = Robot.leftWheel.error;



	Robot.rightWheel.pwmPeriod += Robot.rightWheel.kpGain - Robot.rightWheel.kDGain + Robot.rightWheel.kiGain;
	Robot.leftWheel.pwmPeriod +=  Robot.leftWheel.kpGain - Robot.leftWheel.kDGain + Robot.leftWheel.kiGain;


		Robot.leftWheel.pwmPeriod = limitSpeedMax(Robot.leftWheel.pwmPeriod);
		Robot.rightWheel.pwmPeriod = limitSpeedMax(Robot.rightWheel.pwmPeriod);


		if((Robot.leftWheel.velD == 0) && (abs(Robot.leftWheel.pwmPeriod) < 30))
		{
			setMotor2Speed(2);
		}
		else
		{
			setMotor2Speed(Robot.leftWheel.pwmPeriod);
		}
		if((Robot.rightWheel.velD == 0) && (abs(Robot.rightWheel.pwmPeriod) < 30))
		{
			setMotor1Speed(2);
		}
		else setMotor1Speed(Robot.rightWheel.pwmPeriod);

		Robot.feedbackState =0;

	} // end if of feedback state 1
	else{
		Robot.feedbackState = 1;
	}


}


void stop()
{
	Robot.leftWheel.velD = 0;
	Robot.leftWheel.encoder.clearCount();

	Robot.rightWheel.velD=0;
	Robot.rightWheel.encoder.clearCount();


	Robot.leftWheel.countD=0;
	Robot.rightWheel.countD=0;

}

void setMovement(int movementCounts)
{
	Robot.leftWheel.countD = Robot.leftWheel.encoder.count() + movementCounts;
	Robot.rightWheel.countD = Robot.rightWheel.encoder.count() + movementCounts;
	}


void setLeftMotorSpeed(int encPerPID)
{
	Robot.leftWheel.velD =  encPerPID;
}

void setRightMotorSpeed(int encPerPID)
{
	Robot.rightWheel.velD =  encPerPID;
}



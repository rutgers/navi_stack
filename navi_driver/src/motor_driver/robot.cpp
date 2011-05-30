/*
Robot.c


*/

#include "robot.h"
#include "WProgram.h"
#include <stdint.h>
#include "motorcontrol.h"

#define pos(x) (x>=0)


robotState Robot;

#define DEADBAND 10 
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
	Robot.rightWheel.errorT =0;
	Robot.rightWheel.countA=0;


	Robot.leftWheel.vel = 0;
	Robot.leftWheel.velD = 0;
	Robot.leftWheel.encoder.clearCount();
	Robot.leftWheel.countD = 0;
	Robot.leftWheel.errorp=0;
	Robot.leftWheel.kiGain =0;
	Robot.leftWheel.errorT = 0;
	Robot.leftWheel.countA=0;

	Robot.feedbackState = 0;
	sei();
}


int limitSpeedMax(int speed)
{
	int dead = DEADBAND;
	
	if (speed > 255)  return 255;  // max duty cycle is 510
	if (speed < -255) return -255; // max duty cycle is 510
	
	if ( (speed >0 ) && (speed < dead) )return dead;
	if ( (speed <0) &&  (speed > -dead) ) return -dead;
	return speed;
}


int limit(int lim, int val)
{
        if (val > lim) return lim;  // max duty cycle is 510
        if (val < -lim) return -lim; // max duty cycle is 510
        return val;
}


void updateWheel(wheel& w){
	w.countD += w.velD;
	w.countA += w.vel;

	if ( (w.countD > 200) && (w.countA >200) ) {
		w.countD -= 200;
		w.countA-= 200;
	}
	if (  (w.countD < -200) && (w.countA < -200) ){
		w.countD += 200;
		w.countA += 200;	
	}

}


ISR(TIMER2_OVF_vect)   // feed back loop interrupt
{
	//TCNT2 = 0;      // Feedbackloop modifies every 32.8 miliseconds

	if (Robot.feedbackState == 1){

		Robot.rightWheel.vel = Robot.rightWheel.encoder.count() - Robot.rightWheel.previousCount;

		Robot.leftWheel.vel = Robot.leftWheel.encoder.count() - Robot.leftWheel.previousCount;

		updateWheel(Robot.rightWheel);
		updateWheel(Robot.leftWheel);

		if (Robot.clearEncoder){
			Robot.rightWheel.previousCount = 0;
			Robot.leftWheel.previousCount = 0;
			Robot.rightWheel.encoder.clearCount();
			Robot.leftWheel.encoder.clearCount();
			Robot.clearEncoder = 0;
		}


	Robot.rightWheel.error = Robot.rightWheel.countD - Robot.rightWheel.countA;
	Robot.leftWheel.error = Robot.leftWheel.countD - Robot.leftWheel.countA;

//if (abs(Robot.rightWheel.pwmPeriod) <20) Robot.rightWheel.error  *=2;
//	if (abs(Robot.leftWheel.pwmPeriod) <20) Robot.leftWheel.error  *=2;


	Robot.rightWheel.kpGain = Robot.rightWheel.error*2/5;
	Robot.leftWheel.kpGain = Robot.leftWheel.error*2/5;
	
	
	Robot.rightWheel.kDGain = (Robot.rightWheel.errorp -Robot.rightWheel.error)*7/4;
	Robot.leftWheel.kDGain = (Robot.leftWheel.errorp -Robot.leftWheel.error)*7/4;

	//Robot.leftWheel.errorT += Robot.leftWheel.error/10;
	//Robot.rightWheel.errorT += Robot.rightWheel.error/10;

	//Robot.leftWheel.errorT = limit(20, Robot.leftWheel.errorT);
	//Robot.rightWheel.errorT = limit(20, Robot.rightWheel.errorT);

	Robot.rightWheel.kiGain =0;// Robot.rightWheel.errorT ;
	Robot.leftWheel.kiGain =0;// Robot.leftWheel.errorT;

	
	
	Robot.rightWheel.errorp = Robot.rightWheel.error;
	Robot.leftWheel.errorp = Robot.leftWheel.error;



	Robot.rightWheel.pwmPeriod += Robot.rightWheel.kpGain - Robot.rightWheel.kDGain + Robot.rightWheel.kiGain;
	Robot.leftWheel.pwmPeriod +=  Robot.leftWheel.kpGain - Robot.leftWheel.kDGain + Robot.leftWheel.kiGain;


		Robot.leftWheel.pwmPeriod = limitSpeedMax(Robot.leftWheel.pwmPeriod);
		Robot.rightWheel.pwmPeriod = limitSpeedMax(Robot.rightWheel.pwmPeriod);


		if((Robot.leftWheel.velD == 0) && (abs(Robot.leftWheel.pwmPeriod) < DEADBAND+1) )
		{
			if (pos(Robot.leftWheel.error) != pos(Robot.leftWheel.pwmPeriod) )
			setMotor2Speed(2);
			else  setMotor2Speed(Robot.leftWheel.pwmPeriod);
		}
		else
		{
			setMotor2Speed(Robot.leftWheel.pwmPeriod);
		}
		if((Robot.rightWheel.velD == 0) && (abs(Robot.rightWheel.pwmPeriod) < DEADBAND+1))
		{
		if (pos(Robot.rightWheel.error) != pos(Robot.rightWheel.pwmPeriod) )

                        setMotor1Speed(2);
                        else  setMotor1Speed(Robot.rightWheel.pwmPeriod);

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



/*
 * motorcontrol.c
 *
 *  Created on: Feb 4, 2009
 *      Author: Asher
 */
#include "motorcontrol.h"
#include "WProgram.h"
#include <avr/interrupt.h>
#include <avr/delay.h>


#define motor1Dir 7
#define motor2Dir 8
#define motor1PWM 9
#define motor2PWM 10
#define motor1Enable 11
#define motor2Enable 12



void setMotorVel(int dirPin, int pwmPin, int velocity){
  if (velocity >= 255) velocity = 255;
  if (velocity <= -255) velocity = -255;


  if(velocity <-5){  // Reverse
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, 255+velocity);
  }
  else if(velocity >5 ){ // Forward
    digitalWrite(dirPin,LOW);
    analogWrite(pwmPin, velocity);
  }
  else{
	analogWrite(pwmPin, 0);
	digitalWrite(dirPin, HIGH);
	digitalWrite(pwmPin, HIGH);

  }


}


void setMotor1Speed(int velocity)
{
  setMotorVel(motor1Dir, motor1PWM, velocity);

}

void setMotor2Speed(int velocity){
  setMotorVel(motor2Dir, motor2PWM, velocity);
}

void initMotorDriver()
{
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  pinMode(motor1Enable, OUTPUT);
  pinMode(motor2Enable, OUTPUT);
  digitalWrite(motor1Enable,HIGH);
  digitalWrite(motor2Enable,HIGH);
  setMotor1Speed(0); // make sure the motors are stopped
  setMotor2Speed(0);
}


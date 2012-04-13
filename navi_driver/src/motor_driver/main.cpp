#include "WProgram.h" //include the Arduino library
#include <stdio.h>

#include <avr_ros/Current.h>
#include <avr_ros/MotorCmd.h>
#include "avr_ros/Encoder.h"
#include <avr_ros/ros.h>

#include <avr/interrupt.h>
#include <avr/delay.h>

#include "motorcontrol.h"
#include "robot.h"

//Define global message objects to use in
//the callback functions and throughout the program
ros::Publisher pub_current;
ros::Publisher pub_enc;

navi_driver::Encoder encoder_msg;
navi_driver::Current current_msg;
navi_driver::MotorCmd cmd_msg;
static char update=0;

void toggle()
{ //toggle an led to debug the program
    static char t=0;
    if (!t ) {
        digitalWrite(13, HIGH);   // set the LED on
        t = 1;
    } else {
        digitalWrite(13, LOW);    // set the LED off
        t = 0;
    }
}

void motor_cmd_cb(const ros::Msg* msg){
	toggle();
	setLeftMotorSpeed(cmd_msg.left);
	setRightMotorSpeed(cmd_msg.right);
}

// Since we are hooking into a standard
// arduino sketch, we must define our program in
// terms of the arduino setup and loop functions.
void setup()
{
    Serial.begin(57600);
    pinMode(13, OUTPUT); //set up the LED

   RobotInit();

    pub_current = node.advertise("current");
    pub_enc = node.advertise("encoder");
    node.subscribe("cmd",motor_cmd_cb, &cmd_msg);
}

unsigned long encoder_update_timer =0;
unsigned long current_update_timer =0;
unsigned long read_current_timer=0;

void loop()
{
	 node.spin();

	if (encoder_update_timer < millis()){
		encoder_msg.left = Robot.leftWheel.encoder.count();
		encoder_msg.right = Robot.rightWheel.encoder.count();
		Robot.clearEncoder =1;

		node.publish(pub_enc, &encoder_msg);
		encoder_update_timer = millis()+40;
	}

	if (read_current_timer < millis()){
		current_msg.left += analogRead(0);
		current_msg.right += analogRead(1);
		read_current_timer = millis() + 50;
	}
	if (current_update_timer < millis()){
		current_msg.left  = 20;
		current_msg.right =20;
		node.publish(pub_current,&current_msg);
		current_msg.left =0;
		current_msg.right=0;
		current_update_timer = millis() +1000;
	}

	_delay_ms(1);
}

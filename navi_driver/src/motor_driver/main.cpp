#include "WProgram.h" //include the Arduino library
#include <stdio.h>
#include "PDMotorController.h"
#include <avr_ros/Current.h>
#include <avr_ros/MotorCmd.h>
#include "avr_ros/Encoder.h"
#include <avr_ros/ros.h>

#include <avr/interrupt.h>
#include <avr/delay.h>
#include "encoder_setup.h"

//#include "motor_control.h"
 extern "C" {
   // Get declaration for f(int i, char c, float x)
#include "motor_shb.h"
 }


//Define global message objects to use in
//the callback functions and throughout the program
ros::Publisher pub_current;
ros::Publisher pub_enc;


navi_driver::Encoder encoder_msg;
navi_driver::Current current_msg;
navi_driver::MotorCmd cmd_msg;


static PDMotorController left_motor(2,1, 512,20), right_motor(2,1, 512,20);

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
	mshb_set(0,  cmd_msg.left);
	mshb_set(1,  cmd_msg.right);
	toggle();
}


//Encoder ISR
ISR(PCINT1_vect){
	//toggle();
	left_motor.encoderUpdate(bit_is_set(PINC,PIN2), bit_is_set(PINC, PIN3));
	right_motor.encoderUpdate(bit_is_set(PINC, PIN5), bit_is_set(PINC,PIN4));
}

// Since we are hooking into a standard
// arduino sketch, we must define our program in
// terms of the arduino setup and loop functions.

void setup()
{
    Serial.begin(57600);
    pinMode(13, OUTPUT); //set up the LED

    //Setup encoder and encoder interrupts
    enc_init();
    //Set up motor control pins
    mshb_init();
    mshb_enable(0);
    mshb_enable(1);
    //initMotorDriver();

    //Set up PD control loop timer


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
		encoder_msg.left = left_motor.encoderCount();
		encoder_msg.right = right_motor.encoderCount();
		left_motor.clearEncoder();
		right_motor.clearEncoder();
		node.publish(pub_enc, &encoder_msg);
		encoder_update_timer = millis()+80;
	}

	if (read_current_timer < millis()){
		current_msg.left += analogRead(0);
		current_msg.right += analogRead(1);
		read_current_timer = millis() + 50;
	}
	if (current_update_timer < millis()){
		current_msg.left  /= 20;
		current_msg.right /=20;
		node.publish(pub_current,&current_msg);
		current_msg.left =0;
		current_msg.right=0;
		current_update_timer = millis() +1000;
	}

	_delay_ms(1);

	//for(int i =0; i< 1000; i++);
}

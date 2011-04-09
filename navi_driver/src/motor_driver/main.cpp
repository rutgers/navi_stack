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
#include "pd_setup.h"

extern "C" {
#include "motor_shb.h"
 }


//Define global message objects to use in
//the callback functions and throughout the program
ros::Publisher pub_current;
ros::Publisher pub_enc;


navi_driver::Encoder encoder_msg;
navi_driver::Current current_msg;
navi_driver::MotorCmd cmd_msg;
static char update=0;



static PDMotorController left_motor(4,5, 32000,30), right_motor(4,5 , 32000,30);

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
	mshb_set(0, cmd_msg.left);
	mshb_set(1, cmd_msg.right);
	
	//left_motor.setVelocity(cmd_msg.left);
	//right_motor.setVelocity(cmd_msg.right);
}

//PID ISR
ISR(TIMER2_OVF_vect)   // feed back loop interrupt
{
	TCNT2 = 0;
	if (update){
		TCNT2 = 0;      // Compare time set to ~32 millisecs by setting TCNT2 to 0
		left_motor.PIDUpdate();
		right_motor.PIDUpdate();
		mshb_set(0, left_motor.motorCMD());
		mshb_set(1, right_motor.motorCMD());
		update =0;
	}
	else update =1;

}



//Encoder ISR
ISR(PCINT1_vect){
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
    left_motor.setVelocity(0);
    right_motor.setVelocity(0);

    //Set up PD control loop timer
    //initPIDTimer(); //NOT WORKING

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
		encoder_update_timer = millis()+100;
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
}

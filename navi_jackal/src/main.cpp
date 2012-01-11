/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <util/atomic.h>
#include <WProgram.h>
#include <ros.h>
#include <navi_jackal_msgs/EncoderTicks.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "main.hpp"


static std_msgs::Float32              msg_angvel;
static navi_jackal_msgs::EncoderTicks msg_encoder;

ros::NodeHandle nh;
ros::Publisher pub_angvel("angvel", &msg_angvel);
ros::Publisher pub_encoder("encoder", &msg_encoder);
ros::Subscriber<std_msgs::Float32> sub_angvel_setp("angvel_setp", &change_setpt);

void change_setpt(std_msgs::Float32 const &msg)
{
	// TODO: Listen for separate left and right angular velocities.
	pid_set_target(msg.data, msg.data);
}

void setup(void)
{
	encoder_init();
	motor_init();
	pid_init();

	motor_enable(1);
	//pid_set_target(0.0f, 0.0f);

	nh.initNode();
	nh.advertise(pub_angvel);
	nh.advertise(pub_encoder);
	nh.subscribe(sub_angvel_setp);
}

void loop(void)
{
	ATOMIC_BLOCK (ATOMIC_FORCEON) {
		msg_encoder.ticks_left  = encoder1_buffer;
		msg_encoder.ticks_right = encoder2_buffer;
		encoder1_buffer = 0;
		encoder2_buffer = 0;
	}
	pub_encoder.publish(&msg_encoder);
	nh.spinOnce();
	delay(100);
}

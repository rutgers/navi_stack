/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <util/atomic.h>
#include <WProgram.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "main.hpp"

static std_msgs::Float32 msg_angvel;
static std_msgs::Int32   msg_encoder1;
static std_msgs::Int32   msg_encoder2;

ros::NodeHandle nh;
ros::Publisher pub_angvel("angvel", &msg_angvel);
ros::Publisher pub_encoder1("encoder1", &msg_encoder1);
ros::Publisher pub_encoder2("encoder2", &msg_encoder2);
ros::Subscriber<std_msgs::Float32> sub_angvel_setp("angvel_setp", &change_setpt);

void change_setpt(std_msgs::Float32 const &msg)
{
	// TODO: Listen for separate left and right angular velocities.
	pid_set_target(msg.data, msg.data);
}

void setup(void)
{
	encoder_init();
	//motor_init();
	//pid_init();

	//motor_enable(1);
	//pid_set_target(0.0f, 0.0f);

	nh.initNode();
	nh.advertise(pub_angvel);
	nh.advertise(pub_encoder1);
	nh.advertise(pub_encoder2);
	nh.subscribe(sub_angvel_setp);
}

void loop(void)
{
	ATOMIC_BLOCK (ATOMIC_FORCEON) {
		msg_encoder1.data = motor1_ticks;
		msg_encoder2.data = motor2_ticks;
	}
	pub_encoder1.publish(&msg_encoder1);
	pub_encoder2.publish(&msg_encoder2);
	nh.spinOnce();
	delay(10);
}

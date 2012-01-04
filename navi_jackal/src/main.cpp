/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <util/atomic.h>
#include <WProgram.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "config.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "main.hpp"

static std_msgs::Float32 msg_angvel;
static std_msgs::Int16   msg_encoder;

ros::NodeHandle nh;
ros::Publisher pub_angvel("angvel", &msg_angvel);
ros::Publisher pub_encoder("encoder", &msg_encoder);
ros::Subscriber<std_msgs::Float32> sub_angvel_setp("angvel_setp", &change_setpt);

void change_setpt(std_msgs::Float32 const &msg)
{
}

void setup(void)
{
	encoder_init();
	pid_init();

	nh.initNode();
	nh.advertise(pub_angvel);
	nh.advertise(pub_encoder);
	nh.subscribe(sub_angvel_setp);
}

void loop(void)
{
	ATOMIC_BLOCK (ATOMIC_FORCEON) {
		msg_encoder.data = period_us;
	}
	pub_encoder.publish(&msg_encoder);

	nh.spinOnce();
	delay(2);
}

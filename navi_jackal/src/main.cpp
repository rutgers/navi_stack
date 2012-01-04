/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <WProgram.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include "config.hpp"
#include "encoder.hpp"
#include "main.hpp"

static std_msgs::Float32 msg_angvel;

ros::NodeHandle nh;
ros::Publisher                     pub_angvel("angvel", &msg_angvel);
ros::Subscriber<std_msgs::Float32> sub_angvel_setp("angvel_setp", &change_setpt);

void change_setpt(std_msgs::Float32 const &msg)
{
}

void setup(void)
{
	encoder_init();

	nh.initNode();
	nh.advertise(pub_angvel);
	nh.subscribe(sub_angvel_setp);
}

void loop(void)
{
	nh.spinOnce();
	delay(1000);
}

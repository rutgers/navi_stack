#include <util/atomic.h>
#include <WProgram.h>

#include <ros.h>
#include <navi_jackal/ControlConstants.h>
#include <navi_jackal/EncoderTicks.h>
#include <navi_jackal/VelocitySetpoint.h>
#include <navi_jackal/VoltageSetpoint.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "pid.hpp"

using navi_jackal::EncoderTicks;
using navi_jackal::ControlConstants;
using navi_jackal::VelocitySetpoint;
using navi_jackal::VoltageSetpoint;

static void update_constants(ControlConstants const &msg_consts);
static void update_setpoints(VelocitySetpoint const &msg_setpoints);
static void update_voltages(VoltageSetpoint const &msg_voltages);

static EncoderTicks msg_encoders;

static ros::NodeHandle nh;
static ros::Publisher pub_encoders("encoders", &msg_encoders);
static ros::Subscriber<ControlConstants> sub_constants("constants", &update_constants);
static ros::Subscriber<VelocitySetpoint> sub_setpoints("setpoint", &update_setpoints);
static ros::Subscriber<VoltageSetpoint>  sub_voltages("voltages", &update_voltages);

void setup(void)
{
	encoder_init();
	motor_init();
	pid_init();

	motor_enable(1);

	nh.initNode();
	nh.advertise(pub_encoders);
	nh.subscribe(sub_constants);
	nh.subscribe(sub_setpoints);
}

void loop(void)
{
	ATOMIC_BLOCK (ATOMIC_FORCEON) {
		msg_encoders.ticks_left  = encoder1_buffer;
		msg_encoders.ticks_right = encoder2_buffer;
		encoder1_buffer = 0;
		encoder2_buffer = 0;
	}
	pub_encoders.publish(&msg_encoders);
	nh.spinOnce();
	delay(100);
}

static void update_constants(ControlConstants const &msg_constants)
{
	pid1.kf = pid2.kf = msg_constants.feedforward;
	pid1.kp = pid2.kp = msg_constants.proportional;
	pid1.ki = pid2.ki = msg_constants.integral;
	pid1.kd = pid2.kd = msg_constants.derivative;
	pid1.threshold = pid2.threshold = msg_constants.threshold;
}

static void update_setpoints(VelocitySetpoint const &msg_setpoints)
{
	pid1.enable = true;
	pid2.enable = true;
	pid1.feedforward = msg_setpoints.ff_left;
	pid2.feedforward = msg_setpoints.ff_right;
	pid1.target = msg_setpoints.setpoint_left;
	pid2.target = msg_setpoints.setpoint_right;
}

static void update_voltages(VoltageSetpoint const &msg_voltages)
{
	pid1.enable = false;
	pid2.enable = false;
	motor_set(msg_voltages.pwm_left, msg_voltages.pwm_right);
}

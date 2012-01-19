#include <util/atomic.h>
#include <WProgram.h>

#include <ros.h>
#include <navi_jackal/CalibrationSetpoint.h>
#include <navi_jackal/ControlConstants.h>
#include <navi_jackal/EncoderTicks.h>
#include <navi_jackal/VelocitySetpoint.h>

#include "config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "pid.hpp"

#ifndef JACKAL_CALIBRATION
#warning "Calibration service is disabled, so no encoder ticks will be published."
#endif

using navi_jackal::CalibrationSetpointRequest;
using navi_jackal::CalibrationSetpointResponse;
using navi_jackal::ControlConstants;
using navi_jackal::VelocitySetpoint;

static void update_constants(ControlConstants const &msg_consts);
static void update_setpoints(VelocitySetpoint const &msg_setpoints);
#ifdef JACKAL_CALIBRATION
static void update_calibration(CalibrationSetpointRequest  const &srv_req,
                               CalibrationSetpointResponse       &srv_resp);
#endif

static ros::NodeHandle nh;
static ros::Subscriber<ControlConstants> sub_constants("constants", &update_constants);
//static ros::Subscriber<VelocitySetpoint> sub_setpoints("setpoint", &update_setpoints);
#ifdef JACKAL_CALIBRATION
static ros::ServiceServer<CalibrationSetpointRequest, CalibrationSetpointResponse> srv_calibrate("calibrate", &update_calibration);
#endif

void setup(void)
{
	encoder_init();
	motor_init();
	pid_init();

	motor_enable(1);

	nh.initNode();
	nh.subscribe(sub_constants);
	//nh.subscribe(sub_setpoints);
#ifdef JACKAL_CALIBRATION
	nh.advertiseService(srv_calibrate);
#endif

	nh.loginfo("Jackal Init");
}

void loop(void)
{
	nh.spinOnce();
	delay(100);
}

static void update_constants(ControlConstants const &msg_constants)
{
	for (size_t i = 0; i < PIDS_NUM; i++) {
		pids[i].kf = msg_constants.feedforward;
		pids[i].kp = msg_constants.proportional;
		pids[i].ki = msg_constants.integral;
		pids[i].kd = msg_constants.derivative;
		pids[i].threshold = msg_constants.threshold;
	}
}

static void update_setpoints(VelocitySetpoint const &msg_setpoints)
{
	pids[0].enable = true;
	pids[1].enable = true;
	pids[0].feedforward = msg_setpoints.ff_left;
	pids[1].feedforward = msg_setpoints.ff_right;
	pids[0].target = msg_setpoints.setpoint_left;
	pids[1].target = msg_setpoints.setpoint_right;
}

#ifdef JACKAL_CALIBRATION
static void update_calibration(CalibrationSetpointRequest  const &srv_req,
                               CalibrationSetpointResponse       &srv_resp)
{
	// Disable the PID control loops so they don't fight for the PWMs.
	for (size_t i = 0; i < PIDS_NUM; i++) {
		pids[i].enable = false;
	}
	motor_set(srv_req.pwm_left, srv_req.pwm_right);

	// Count the number of encoder ticks since the last service response. We
	// need to be careful here because the ticks are incremented in an ISR and
	// accessing 32-bit data is non-atomic. Note that the calibration code does
	// not modify ticks_short, which is used by the PID controller.
	ATOMIC_BLOCK (ATOMIC_FORCEON) {
		srv_resp.ticks_left  = encoders[0].ticks_long;
		srv_resp.ticks_right = encoders[1].ticks_long;
		encoders[0].ticks_long = 0;
		encoders[1].ticks_long = 0;
	}
}
#endif

#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('navi_jackal')
import rospy

class Calibrator:
    def __init__(self, dur_delay, dur_sample):
        self.pub_pwm = rospy.Publisher('voltages', VoltageSetpoint)
        self.sub_enc = rospy.Subscriber('encoders', EncoderTicks, self.encoder_tick)
        self.ticks = 0

        self.pid_freq = rospy.get_param('~pid_frequency')
        self.pwm_max  = rospy.get_param('~pwm_max')

        self.dur_delay  = rospy.Duration.from_sec(dur_delay)
        self.dur_sample = rospy.Duration.from_sec(dur_sample)
        self.table_left  = list()
        self.table_right = list()

    def encoders_tick(self, encoders):
        self.ticks_left  += encoders.ticks_left
        self.ticks_right += encoders.ticks_right

    def encoders_clear(self):
        self.ticks_left  = 0
        self.ticks_right = 0

    def scale_omega(self, ticks):
        return ticks / (self.dur_sample * self.pid_freq)

    def spin_sleep(self, duration):
        rate  = rospy.Rate(self.pid_freq * 2)
        begin = rospy.get_rostime()
        end   = begin + self.dur_delay

        while rospy.get_rostime() < end:
            rospy.spinOnce()
            r.sleep()

    def evaluate_pwm(self, pwm):
        msg_pwm = VoltageSetpoint(pwm_left = pwm, pwm_right = pwm)
        self.pub_pwm.publish(msg_pwm)

        self.spin_sleep(self.dur_delay)
        self.encoders_clear()
        self.spin_sleep(self.dur_sample)

        omega_left  = self.scale_omega(self.ticks_left)
        omega_right = self.scale_omega(self.ticks_right)

        self.table_left.append(omega_left)
        self.table_right.append(omega_right)

    def stop(self):
        msg_pwm = VoltageSetpoint(pwm_left = 0, pwm_right = 0)
        self.pub_pwm.publish(msg_pwm)

def main():
    rospy.init_node('calibrate')

    try:
        (dur_delay, dur_sample, pwm_step) = rospy.myargv()
    except ValueError:
        rospy.logerr('Incorrect number of arguments.')
        rospy.logerr('usage: ./step.py <delay> <sample> <pwm step>')
        rospy.signal_shutdown('Incorrect number of arguments.')

    node = Calibrator(dur_delay, dur_sample)
    for pwm in range(-pwm_max, pwm_max + 1, pwm_step):
        node.evaluate_pwm(pwm)
    node.stop()

    print('Left:  ', node.table_left)
    print('Right: ', node.table_right)

if __name__ == '__main__':
    main()

# vim: set et:set sw=4 ts=4:

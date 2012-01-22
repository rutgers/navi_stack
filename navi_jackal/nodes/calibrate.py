#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('navi_jackal')
import rospy
import csv
import sys

from navi_jackal.srv import CalibrationSetpoint

class Calibrator:
    def __init__(self, dur_delay, dur_sample):
        self.srv_pwm = rospy.ServiceProxy('calibrate', CalibrationSetpoint)
        self.ticks_left  = 0
        self.ticks_right = 0

        self.pid_freq = rospy.get_param('~pid_frequency')

        self.dur_delay  = dur_delay
        self.dur_sample = dur_sample
        self.table_left  = list()
        self.table_right = list()

    def set_pwm(self, pwm):
        response = self.srv_pwm(pwm, pwm)
        omega_left  = self.scale_omega(response.ticks_left)
        omega_right = self.scale_omega(response.ticks_right)
        return (omega_left, omega_right)

    def scale_omega(self, ticks):
        return ticks / (self.dur_sample * self.pid_freq)

    def evaluate(self, pwms):
        table = list()
        pwm = pwms[0]

        rospy.wait_for_service('calibrate')

        for pwm_next in pwms[1:] + [ 0 ]:
            # Setting the same speed twice is intentional because it clears the
            # current count of encoder ticks. This delay gives the motor time
            # to reach its steady-state velocity.
            print('PWM = {0}'.format(pwm), file=sys.stderr)
            self.set_pwm(pwm)
            rospy.sleep(self.dur_delay)
            self.set_pwm(pwm)

            # Calculate the average number of ticks per PID loop over the
            # sample duration.
            rospy.sleep(self.dur_sample)
            omega_left, omega_right= self.set_pwm(pwm_next)
            table.append((pwm, omega_left, omega_right))
            pwm = pwm_next

        return table

def main():
    rospy.init_node('calibrate')

    try:
        args = rospy.myargv()
        dur_delay  = float(args[1])
        dur_sample = float(args[2])
        pwm_step   = int(args[3])
    except IndexError:
        rospy.logerr('Incorrect number of arguments.')
        rospy.logerr('usage: ./step.py <delay> <sample> <pwm step>')
        rospy.signal_shutdown('Incorrect number of arguments.')
        return 1
    except ValueError:
        rospy.logerr('One or more arguments has an invalid type.')
        rospy.signal_shutdown('One or more arguments has an invalid type.')
        return 1

    pwm_max = rospy.get_param('~pwm_max')
    pwms    = range(-pwm_max, pwm_max + 1, pwm_step)

    # Generate a three-column table of left and right angular velocity versus
    # input PWM value (i.e. voltage).
    node   = Calibrator(dur_delay, dur_sample)
    table  = node.evaluate(pwms)
    writer = csv.writer(sys.stdout)
    writer.writerows(table)
    return 0

if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)

# vim: set et:set sw=4 ts=4:

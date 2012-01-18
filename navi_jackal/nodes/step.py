#!/usr/bin/env python

import roslib; roslib.load_manifest('navi_jackal')
import rospy

from geometry_msgs.msg import Twist

def main():
    rospy.init_node('step')

    try:
        (vel1, vel2, step) = rospy.myargv()
    except ValueError:
        rospy.logerr('Incorrect number of arguments.')
        rospy.logerr('usage: ./step.py <before> <after> <time>')
        rospy.signal_shutdown('Incorrect number of arguments.')

    twist     = Twist()
    pub_twist = rospy.Publisher('velocity', Twist)

    # Use velocity V1 for t < t_step.
    twist.linear.x = vel1
    pub_twist.publish(twist)
    rospy.sleep(step)

    # Switch to velocity V2 when t >= t_step.
    twist.linear.x = vel2
    pub_twist.publish(twist)
    rospy.sleep(step)

    # Stop the motors before exiting.
    twist.linear.x = 0.0
    pub_twist.publish(twist)

if __name__ == '__main__':
    main()

# vim: set et:set sw=4 ts=4:

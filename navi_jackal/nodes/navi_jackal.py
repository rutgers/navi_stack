#!/usr/bin/env python

import roslib; roslib.load_manifest('navi_jackal')
import rospy

from std_msgs.msg import Float64
from navi_jackal.cfg import JackalConfig
from navi_jackal.msg import EncoderTicks
from dynamic_reconfigure.server import Server

class JackalNode:
    def __init__(self):
        rospy.init_node('navi_jackal')
        self.pub_angvel_left  = rospy.Publisher('omega_left', Float64)
        self.pub_angvel_right = rospy.Publisher('omega_right', Float64)
        self.sub_encoders     = rospy.Subscriber('encoders', EncoderTicks, self.tick)
        self.kp = rospy.get_param('~p')
        self.ki = rospy.get_param('~i')
        self.kd = rospy.get_param('~d')

        self.constants = rospy.get_param('~')

        assert 'p' in self.pid
        assert 'i' in self.pid
        assert 'd' in self.pid

    def reconfigure(self, config, level):
        pass

    def spin(self):
        rospy.spin()

    def tick(self, msg_encoder):
        omega_left  = msg_encoder.ticks_left * 
        omega_right = msg_encoder.ticks_right

if __name__ == '__main__':
    try:
        node = JackalNode()
        conf = Server(JackalConfig, node.reconfigure)
        node.spin()
    except rospy.ROSInterruptException:
        pass

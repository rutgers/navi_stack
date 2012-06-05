#!/usr/bin/env python
import roslib; roslib.load_manifest('navi_calibration')
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from stereo_plane.msg import Plane

def callback(msg):
    pub.publish(Plane(
        header = Header(
            stamp = msg.header.stamp,
            frame_id = fixed_frame_id
        ),
        point  = Vector3(0, 0, 0),
        normal = Vector3(0, 0, 1),
        type   = Plane.TYPE_DEFAULT
    ))

def main():
    global fixed_frame_id, pub
    rospy.init_node('static_ground')
    fixed_frame_id = rospy.get_param('~frame_id', '/base_footprint')
    sub = rospy.Subscriber('camera_info', CameraInfo, callback)
    pub = rospy.Publisher('ground_plane', Plane)
    rospy.spin()

if __name__ == '__main__':
    main()

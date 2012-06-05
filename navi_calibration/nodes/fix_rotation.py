#!/usr/bin/env python
import roslib; roslib.load_manifest('navi_calibration')
import rospy, math, tf
import numpy as np
from tf.msg import tfMessage
from sensor_msgs.msg import Imu

interesting_frames = { '/laser', '/camera_left_optical', '/camera_right_optical' }

def callbackCompass(msg):
    qt_raw = np.array([ msg.orientation.x, msg.orientation.y,
                        msg.orientation.z, msg.orientation.w ])
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(qt_raw)
    roll  = -(roll + math.pi)
    qt = tf.transformations.quaternion_from_euler(roll, pitch, 0.0)
    pub_tf.sendTransform((0, 0, 0), qt, msg.header.stamp,
                         base_frame_id + suffix, base_frame_id)

    # Prevent main() from falling back on the default transformation.
    global sent
    sent = True

def callbackTf(msg):
    global interesting_frames
    interesting = False

    for transform in msg.transforms:
        if (transform.child_frame_id in interesting_frames and
           not transform.child_frame_id.endswith(suffix)):
            # Recursively add parents to the set of interesting frames.
            if transform.header.frame_id != base_frame_id:
                interesting_frames.add(transform.header.frame_id)

            transform.header.frame_id += suffix
            transform.child_frame_id += suffix
            interesting = True

    if interesting:
        pub_tfmsg.publish(msg)

def sendDefaultTransform():
    pub_tf.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.get_rostime(),
                         base_frame_id + suffix, base_frame_id)

def main():
    global base_frame_id, rotated_frame_id, interesting_frames, suffix, sent
    global pub_tf, pub_tfmsg
    rospy.init_node('fix_rotation')

    
    rate = rospy.get_param('~rate', 0.1)
    base_frame_id = rospy.get_param('~base_frame', '/base_footprint')
    rotated_frame_id = rospy.get_param('~rotated_frame', '/base_footprint_rotated')
    suffix = rospy.get_param('~suffix', '_rotated')

    sub_tfmsg = rospy.Subscriber('/tf', tfMessage, callbackTf)
    pub_tfmsg = rospy.Publisher('/tf', tfMessage)

    sub_compass = rospy.Subscriber('compass', Imu, callbackCompass)
    sub_tf = rospy.Subscriber('tf', tfMessage)
    pub_tf = tf.TransformBroadcaster()

    r = rospy.Rate(rate)
    sent = False
    while not rospy.is_shutdown():
        if not sent:
            rospy.logwarn('broadcasting default transformation')
            sendDefaultTransform()
        sent = False
        r.sleep()

if __name__ == '__main__':
    main()

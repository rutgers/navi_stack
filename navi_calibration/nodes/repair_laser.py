#!/usr/bin/env python
import roslib; roslib.load_manifest('navi_calibration')
import rospy
from sensor_msgs.msg import LaserScan

def callback(scan):
    for i in xrange(0, len(scan.ranges)):
        if scan.ranges[i] < scan.range_min:
            scan.ranges[i] = scan.range_max
    pub.publish(scan)

def main():
    global pub
    sub = rospy.Subscriber('laser', LaserScan)
    pub = rospy.Publisher('laser_fixed', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

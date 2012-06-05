#!/usr/bin/env python
import roslib; roslib.load_manifest('navi_calibration')
import rospy
from sensor_msgs.msg import LaserScan

def callback(scan):
    scan.ranges = list(scan.ranges)
    for i in xrange(0, len(scan.ranges)):
        if scan.ranges[i] < scan.range_min:
            scan.ranges[i] = scan.range_max
    pub.publish(scan)

def main():
    global pub
    rospy.init_node('repair_laser')
    sub = rospy.Subscriber('laser', LaserScan, callback)
    pub = rospy.Publisher('laser_fixed', LaserScan)
    rospy.spin()

if __name__ == '__main__':
    main()

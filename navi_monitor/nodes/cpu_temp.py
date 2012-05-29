#!/usr/bin/env python
import roslib; roslib.load_manifest('navi_monitor')
import rospy
import re
from subprocess import Popen, PIPE
from std_msgs.msg import Float64

def get_cpu_temp():
    process = Popen(['sensors'], stdout=PIPE)
    pattern = re.compile('^Core \d+:\s+\+(\d+\.\d+)')
    temperatures = list()

    for line in process.stdout:
        match = pattern.match(line)
        if match:
            try:
                temperature = float(match.group(1))
                temperatures.append(temperature)
            except ValueError:
                return None

    if temperatures:
        return max(temperatures)
    else:
        return None

def main():
    rospy.init_node('navi_monitor')
    pub_temp = rospy.Publisher('temperature/cpu', Float64)
    rate = rospy.get_param('~rate', 1.0)

    while not rospy.is_shutdown():
        temperature = get_cpu_temp()
        if temperature:
            pub_temp.publish(Float64(temperature))
        else:
            rospy.logwarn('Unable to measure CPU temperature.')
        rospy.sleep(rate)

if __name__ == '__main__':
    main()

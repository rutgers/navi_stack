#! /usr/bin/env python

import roslib; roslib.load_manifest('novatel_gps')
from nav_msgs.msg import Odometry

import serial
import rospy

def parseBESTUTMA(raw):
    # Split the NMEA header from the actual message.
    try:
        nmea_raw, utm_raw = raw.split(';')
        nmea_list = nmea_raw.split(',')
        utm_list  = utm_raw.split(',')
    except ValueError:
        rospy.logwarn('invalid message')
        return None

    # Basic sanity checking before we unpack the message.
    if len(nmea_list) != 10:
        rospy.logwarn('message header has {0} fields; expected {1}', len(nmea_list), 10)
        return None
    elif len(nmea_list) < 1 or nmea_list[0] != '#BESTUTMA':
        rospy.logwarn('message does not of type BESTUTMA')
        return None
    elif len(utm_list) != 23:
        rospy.logwarn('BESTUTMA message has {0} fields; expected {1}', len(utm_list), 23)
        return None
    
    # Unpack the BESTUTMA message.
    try:
        sol_status, sol_type = utm_raw[0], utm_raw[1]
        northing, easting = float(utm_raw[4]), float(utm_raw[5])
        sigma_northing, sigma_easting = float(utm_raw[9]), float(utm_raw[10])
    except ValueError:
        rospy.logwarn('BESTUTMA message contains invalid reading')
        return None

    if sol_status != 'SOL_COMPUTED':
        rospy.logwarn('no solution computed')
        return None
    elif sol_type != 'OMNISTAR_HP':
        rospy.logwarn('not receiving OmniSTAR corrections')
        return None

    return northing, easting, sigma_northing, sigma_easting

def generateOdomMsg(northing, easting, sigma_northing, sigma_easting):
    big = 99999.0
    return Odometry(
        header = Header(
            stamp = rospy.Time.now(),
            frame_id = '/base_link'
        ),
        pose = PoseWithCovariance(
            pose = Pose(
                position = Point(
                    x = easting,  # TODO: Compute a difference.
                    y = northing, # TODO: Compute a difference.
                    z = 0.0
                )
            ),
            covariance = [
                sigma_easting, 0.0,            0.0, 0.0, 0.0, 0.0,
                0.0,           sigma_northing, 0.0, 0.0, 0.0, 0.0,
                0.0,           0.0,            big, 0.0, 0.0, 0.0,
                0.0,           0.0,            0.0, big, 0.0, 0.0,
                0.0,           0.0,            0.0, 0.0, big, 0.0,
                0.0,           0.0,            0.0, 0.0, 0.0, big
            ]
        ),
        twist = TwistWithCovariance(
            covariance = [
                -1, 0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
            ]
        )
    )

if __name__ == '__main__':
	rospy.init_node('gps_utm')
	port_name= '/dev/gps_novatel'
	
	print "Opening up ", port_name
	
	port = serial.Serial(port_name, 115200, timeout=0.75)
		
	pub = rospy.Publisher('/gps/odom', Odometry)
	
	while not rospy.is_shutdown():
		try:
			line = port.readline()
			odom = parseBESTUTMA(line)
			if (odom != None):
				pub.publish(odom)
		except:
			pass

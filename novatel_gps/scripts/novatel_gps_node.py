#!/usr/bin/env python
import roslib; roslib.load_manifest('novatel_gps')
import rospy, serial
from nav_msgs.msg import Odometry

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
        sol_status, sol_type = utm_list[0], utm_list[1]
        northing, easting = float(utm_list[4]), float(utm_list[5])
        sigma_northing, sigma_easting = float(utm_list[9]), float(utm_list[10])
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
    se = sigma_easting
    sn = sigma_northing
    return Odometry(
        header = Header(
            stamp = rospy.Time.now(),
            frame_id = '/base_link'
        ),
        pose = PoseWithCovariance( pose = Pose(
                position = Point(
                    x = easting,
                    y = northing,
                    z = 0.0
                )
            ),
            covariance = [
                se, 0,  0,   0,   0,   0,
                0,  sn, 0,   0,   0,   0,
                0,  0,  big, 0,   0,   0,
                0,  0,  0,   big, 0,   0,
                0,  0,  0,   0,   big, 0,
                0,  0,  0,   0,   0,   big
            ]
        ),
        twist = TwistWithCovariance(
            covariance = [
                -1, 0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0,
                0,  0, 0, 0, 0, 0
            ]
        )
    )

if __name__ == '__main__':
    rospy.init_node('novatel_gps_node')
    fp = serial.Serial(
        port     = rospy.get_param('~port'),
        baudrate = rospy.get_param('~baud'),
        parity   = serial.PARITY_NONE,
        stopbits = 1,
        timeout  = None
    )

    while not rospy.is_shutdown():
        try:
            line = port.readline()
            data = parseBESTUTMA(line)
            print data
            #if (odom != None):
            #    pub.publish(odom)
        except:
            pass

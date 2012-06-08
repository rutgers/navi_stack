#!/usr/bin/env python
import roslib; roslib.load_manifest('novatel_gps')
import rospy, serial
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, TwistWithCovariance

prev_sol_status = 'SOL_COMPUTED'
prev_sol_type = 'OMNISTAR_HP'

def parseBESTUTMA(raw):
    global prev_sol_status
    global prev_sol_type

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

    # Warn if we we have a bad fix or are missing OmniSTAR corrections.
    if sol_status != 'SOL_COMPUTED':
        if sol_status != prev_sol_status:
            rospy.logwarn('No solution computed.')
    elif sol_type != 'OMNISTAR_HP':
        if sol_type != prev_sol_type:
            rospy.logwarn('Not receiving OmniSTAR HP corrections.')

    prev_sol_status = sol_status
    prev_sol_type = sol_type

    if sol_status == 'SOL_COMPUTED':
        return northing, easting, sigma_northing, sigma_easting
    else:
        return None

def generateOdomMsg(northing, easting, sigma_northing, sigma_easting):
    big = 99999.0
    se = sigma_easting
    sn = sigma_northing
    return Odometry(
        header = Header(
            stamp = rospy.Time.now(),
            frame_id = global_frame_id
        ),
        child_frame_id = frame_id,
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
    pub_gps = rospy.Publisher('gps', Odometry)

    fp = serial.Serial(
        port     = rospy.get_param('~port'),
        baudrate = rospy.get_param('~baud'),
        parity   = serial.PARITY_NONE,
        stopbits = 1,
        timeout  = None
    )
    frame_id = rospy.get_param('~frame_id', '/base_link')
    global_frame_id = rospy.get_param('~global_frame_id', '/map')
    rate = rospy.get_param('~inv_rate_per_sec', 1)

    fp.write('unlogall com1\r\nlog bestutma ontime {0}\r\n'.format(rate))
    fp.flushOutput()
    fp.flushInput()

    while not rospy.is_shutdown():
        line = fp.readline()
        data = parseBESTUTMA(line)
        if not data: continue

        odom = generateOdomMsg(*data)
        if odom:
            pub_gps.publish(odom)

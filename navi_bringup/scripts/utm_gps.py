#! /usr/bin/env python

import roslib; roslib.load_manifest('navi_bringup')
from gps_common.msg import *
from nav_msgs.msg import Odometry

import serial
import rospy

ex_string = """#OMNIHPPOSA,COM3,0,24.5,FINESTEERING,1638,514615.000,00000000,808d,6302;SOL_COMPUTED,OMNISTAR_HP,42.67895919957,-83.19594794881,283.4438,-35.0000,WGS84,1.0071,0.9472,1.0960,"1001",6.000,0.
000,15,9,9,9,0,00,0,03*f6757735
"""

ex_utm_string= """
#BESTUTMA,COM3,0,25.5,FINESTEERING,1638,515217.200,00000000,eb16,6302;SOL_COMPUTED,OMNISTAR_HP,17,T,4727502.1668,320079.3993,284.4611,-35.0000,WGS84,0.0984,0.4398,0.1960,"1001",4.000,0.000,15,8,8,8,0,00,0,03*11855669
"""

start_x = None
start_y = None
start_z = None


def parseOMNIHPPOSA(gps_string):
	fix = gps_common.msg.GPSFix()
	fix.header.stamp="/base_link"
	fix.header.frame_id="/base_link"
	fix.header.stamp= rospy.Time.now()
	
	pass
def parseBESTUTMA(  gps_string):
	global start_x
	global start_y
	#check to see that it was BESTUMA
	
	gps_string = gps_string.lstrip()
	gps_string = gps_string.rstrip()
	
	
	if ('#BESTUTMA' != gps_string[:9]):
		return None
		
	#check checksum
	
	#now find UTM and compose the odometry msg
	
	odom = Odometry()
	odom.header.stamp = rospy.Time.now()
	odom.header.frame_id = '/base_link'

	tokens = gps_string.split(',')

	northing = float(tokens[13])
	easting = float(tokens[14])
	print "Solution_type", tokens[10], "  -  ".   "Northing ", northing , "Easting ", easting 
	
	
	if (start_x != None):
		odom.pose.pose.position.x =  easting - start_x
		odom.pose.pose.position.y =  northing - start_y 
	else:
		start_x = easting
		start_y = northing
		return None
	
	cov_easting = float(tokens[19])
	cov_northing = float(tokens[18])
	
	cov_twist=[	9999999 , 0 , 0 , 0, 0, 0, #fake covariance
				0 , 99999 , 0 , 0, 0, 0,  #must get real covariances..
				0 , 0 , 99999 , 0, 0, 0, 
				0 , 0 , 0 , 99999, 0, 0, 
			    0 , 0 , 0 , 0, 99999, 0, 
				0 , 0 , 0 , 0, 0, 99999]
						   
	cov_pos = [cov_easting , 0 , 0 , 0, 0, 0, 
			   0 , cov_northing , 0 , 0, 0, 0, 
			   0 , 0 , 99999 , 0, 0, 0, 
			   0 , 0 , 0 , 99999, 0, 0, 
			   0 , 0 , 0 , 0, 99999, 0, 
			   0 , 0 , 0 , 0, 0, 99999]
			   
	odom.twist.covariance = cov_twist
	odom.pose.covariance  = cov_pos
	
	
	return odom
		
	

if __name__ == '__main__':
	rospy.init_node('gps_utm')
	port_name= '/dev/gps_novatel'
	
	print "Opening up ", port_name
	
	#port = serial.Serial(port_name, 115200)
	
	print parseBESTUTMA(ex_utm_string)
	print parseBESTUTMA(ex_utm_string)

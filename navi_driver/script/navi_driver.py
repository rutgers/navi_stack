#! /usr/bin/env python

import roslib; roslib.load_manifest('navi_driver')
import rospy

import navi_driver.msg
from navi_driver.msg import Encoder, MotorCmd

import nav_msgs.msg
from nav_msgs.msg import Odometry

import tf

import navi_driver.diff_model 
from navi_driver.diff_model import DiffDriveModel

import geometry_msgs.msg
from geometry_msgs.msg import Twist, Quaternion



def constructOdomMessage(diff_model, tf_frame, tf_child):
	odom =  Odometry()
	odom.header.stamp = rospy.get_rostime();
	odom.header.frame_id = tf_frame
	odom.child_frame_id = tf_child
			
	x,y,z,theta = diff_model.position()
	odom.pose.pose.position.x = x
	odom.pose.pose.position.y = y
	odom.pose.pose.position.z = 0.0
        
	qNpy =   tf.transformations.quaternion_from_euler(0,0,theta)
	odom_quat = Quaternion(qNpy[0], qNpy[1], qNpy[2], qNpy[3])
	odom.pose.pose.orientation = odom_quat
        
        #print "Odom values :" , xOdom,  yOdom, theta
        #calculate velocity
	vx = diff_model.vel
	vy = 0
	vth = diff_model.dtheta
        
	odom.twist.twist.linear.x = vx
	odom.twist.twist.linear.y = vy
	odom.twist.twist.angular.z = vth
		
	odom.twist.covariance = diff_model.cov_twist
	odom.pose.covariance = diff_model.cov_pos
	return odom


last_encoder_update = 9999999.99999

def encoder_cb(model, odom_pub, tfBroad, msg):
	"""
	"""
	global last_encoder_update
	now = rospy.Time.now()
	dt =now.to_sec() - last_encoder_update
	last_encoder_update = now.to_sec()
	
	model.update(msg.left, msg.right, dt)
	
	#now publish odom msg and tf
	odom = constructOdomMessage(model, 'odom', 'drive_footprint')
	odom_pub.publish(odom)
	
	#now broadcast tf..
	x,y,z, theta = model.position()
	tfBroad.sendTransform((x, y, 0),
		tf.transformations.quaternion_from_euler(0, 0, model.theta),rospy.Time.now(),
                     'odom',
                     "drive_footprint")
	
def cmd_vel_cb(model,velT_units, motor_pub, cmd_vel_msg):
	"""
	"""
	angular = cmd_vel_msg.angular.z
	forwardVel = cmd_vel_msg.linear.x

	lw, rw = model.getEncoderCmd(forwardVel, angular)
	motor_msg = MotorCmd()
	motor_msg.left = lw*velT_units
	motor_msg.right = rw*velT_units
	motor_pub.publish(motor_msg)
	

if __name__ == '__main__':
	rospy.init_node('navi_driver')
	rospy.loginfo('Starting Navi Driver')
	drive_diameter = rospy.get_param('~drive_diameter')
	lw_encoder_to_meter = rospy.get_param('~lw_encoder_to_meter')
	rw_encoder_to_meter = rospy.get_param('~rw_encoder_to_meter')
	
	#the cmd velocity of motors is in units of encoder ticks
	# per PID cycle
	# as a result, since the PID is 30.5 hz, the real cmd vel is 
	# MotorCMD/velocity_units_secs - MotorCMD*30.5
	velocity_units_secs = float (rospy.get_param('~velocity_units_secs') )

	model = DiffDriveModel(drive_diameter, lw_encoder_to_meter, rw_encoder_to_meter)
	
	pub_odom= rospy.Publisher('odom', Odometry)
	tfBroad = tf.broadcaster.TransformBroadcaster()
	
	sub_encoder = rospy.Subscriber('encoder', Encoder, lambda msg : encoder_cb(model,pub_odom, tfBroad,msg))
	
	pub_motor_cmd = rospy.Publisher('motor_cmd', MotorCmd)
	sub_cmd_vel =rospy.Subscriber("cmd_vel", Twist, lambda msg: cmd_vel_cb(model, velocity_units_secs, pub_motor_cmd, msg) )
	
	rospy.spin()
	

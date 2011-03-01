#! /usr/bin/env python
"""
This script is responsible for wrapping the avr_bridge 
generated driver. 
	converts the geometry_twist messages to wheel speeds
	encoder readings  to odometry_msg
"""

import roslib; roslib.load_manifest('navi_driver')

import rospy
import piper_driver.msg
import geometry_msgs.msg
import nav_msgs.msg
import tf

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist
from piper_driver.msg import Encoder
from piper_driver.msg import Drive


import time

from math import pi

#Differential Drive constants

rotational_ff = 1
driveDiameter_ff = 1

WheelDiameter = 0.1016
driveBaseDiameter = 0.5


#
piperX=0
piperY=0
piperTheta =0

piperVelL = 0
piperVelR = 0


def differentialDriveOdomUpdate(driveDiameter, leftDist, rightDist, pX, pY, pTheta):
    """Takes the distance traveled by each side of the robot, updates the previous position and orientation,
    and returns the tuple x,y, theta"""
    distance = (leftDist+ rightDist)/2.0
    thetaInc = (rightDist - leftDist)/driveDiameter
    theta = pTheta + thetaInc
    theta = theta%(2*math.pi)
    x = pX + math.cos(theta)* distance 
    y = pY + math.sin(theta) * distance
    return x,y,theta



class RobotWrapper():
	'''
	This class controls the interface to Navi's Driver. It acts as the interface to the robot.
	
	'''
	def __init__(self, diffDiameter, encoderToMeters):
		"""
        Sets up the robot, but does not connect to the robot
		"""
		self.driveDiameter = diffDiameter# width of the robots drive train, important for calculating turns
		self.leftWheelVelocity = 0  #velocity in meters/sec
		self.rightWheelVelocity =0  #velocity in meters/sec
		self.xOdom =0  #meters
		self.yOdom =0  #meters
		self.theta = 0 #radians
		self.odomVelX =0
		self.odomVelY =0
		self.angVel =0
		self.Lm =0
		self.Rm =0
		self.velX =0

		self.__LastOdomTime = time.time()
		self.__encoderPulsesPerMeter = encoderToMeters
        
		self.pub_odom = rospy.Publisher('odom', Odometry)
		self.pub_drive = rospy.Publisher('drive', Drive)

		self.sub_cmd_vel =rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)
		self.sub_encoder =rospy.Subscriber("encoder", Encoder, self.encoder_cb)
		self.broadcastTF = rospy.get_param('publish_odom_tf', False)
		self.tfBroadcaster = tf.broadcaster.TransformBroadcaster()
		
		self.last_cmd_time = time.time()
		self.last_cmd_drive  = Drive()
          
	def setWheelSpeed(self,left, right):
		"""Sets the Wheel velocities of the robot. (Assumes differential drive, )
            Arguments of are in meters
            no return value
		"""
		drive_msg  = Drive()
		drive_msg.leftWheel  = self.__metersToEncoders(left)
		drive_msg.rightWheel = self.__metersToEncoders(right)
		self.last_cmd_time = time.time()
		self.last_cmd_drive = drive_msg
                
	def encoder_cb(self, encoder_msg):
		"""Updates the robot classes odometry variables with the xDist, ydist and theta"""
		left, right = encoder_msg.leftWheel, encoder_msg.rightWheel
        
		Lm, Rm = self.__encoderToMeters(float(left)), self.__encoderToMeters(float(right))

		x,y, theta  = differentialDriveOdomUpdate(self.driveDiameter  , Lm, Rm, self.xOdom , self.yOdom, self.theta)
        
		now = time.time()
		dif = now- self.__LastOdomTime
		self.__LastOdomTime = now
        
		self.odomVelX = (x - self.xOdom)/dif
		self.odomVelY = (y - self.yOdom)/dif
		self.angVel= (self.theta - theta)/dif
        
        
		self.velX = ((Lm + Rm)/2)/dif
        
        
		self.xOdom, self.yOdom, self.theta = x,y,theta
        
		self.publishPosition()
 
	def __encoderToMeters(self, encoderValue):
		"""Converts encoder values to meters
		"""
		return encoderValue/ self.__encoderPulsesPerMeter
        
	def __metersToEncoders(self, meters):
		return meters* self.__encoderPulsesPerMeter

	def calculateWheelVel(self, cmdVel):
		"""Calculates wheel velocity in meters per second from a cmd_velocity ROs message"""
		angular = cmdVel.angular.z
		forwardVel = cmdVel.linear.x
		angularDif = angular * self.driveDiameter
		leftVel =  (forwardVel-angularDif/2)
		rightVel = (angularDif/2 + forwardVel)
		

		return leftVel, rightVel

	def cmd_vel_cb(self, cmd_vel):
		
		cmd_vel.linear.x *= 1/(16.0)/3
		self.cmd_vel = cmd_vel
		left, right = self.calculateWheelVel(cmd_vel)
				
		self.setWheelSpeed(left, right)
		 
	def  publishPosition(self):
		"""Publishes the position by constructing an
			odometry message and broadcasting the tf between odom
			and the base_footprint"""
		xOdom = self.xOdom
		yOdom = self.yOdom
		theta = self.theta
    
		odom =  Odometry()
		odom.header.stamp = rospy.get_rostime();
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_footprint"
        
		odom.pose.pose.position.x = xOdom
		odom.pose.pose.position.y = yOdom
		odom.pose.pose.position.z = 0.0
        
		qNpy =   tf.transformations.quaternion_from_euler(0,0,theta)
		odom_quat = Quaternion(qNpy[0], qNpy[1], qNpy[2], qNpy[3])
		odom.pose.pose.orientation = odom_quat
        

		odom.twist.twist.linear.x = self.velX
		odom.twist.twist.linear.y = 0
		odom.twist.twist.angular.z = self.angVel
        
        
        #fake covariances
		cov = [0.003 , 0 , 0 , 0, 0, 0, 
        	   0 , 0.003 , 0 , 0, 0, 0, 
        	   0 , 0 , 99999 , 0, 0, 0, 
        	   0 , 0 , 0 , 99999, 0, 0, 
        	   0 , 0 , 0 , 0, 99999, 0, 
        	   0 , 0 , 0 , 0, 0, 0.001]
		odom.twist.covariance = cov
		odom.pose.covariance = cov
		if (self.broadcastTF):
			self.tfBroadcaster.sendTransform((xOdom,yOdom,0), qNpy, rospy.Time.now(), 'base_footprint', 'odom')
			
		self.pub_odom.publish(odom)
		



if __name__ == "__main__":
	rospy.init_node('piper_bridge_wrapper')
	piper = RobotWrapper(driveBaseDiameter, 1058.62491)
	rospy.loginfo("Successfully started Piper Driver Wrapper")

	while(not rospy.is_shutdown()):
		piper.pub_drive.publish(piper.last_cmd_drive)
		rospy.sleep(0.05)

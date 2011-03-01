import roslib; roslib.load_manifest('navi_driver')

import numpy as np
import math

class DiffDriveModel:
	"""
	Model of a Differential drive robot
	"""
	def init(self, driver_diameter, lw_encoder_per_meter, rw_encoder_per_meter):
		self.lw_encoder_per_meter= float(lw_encoder_per_meter)
		self.rw_encoder_per_meter= float(rw_encoder_per_meter)
		
		self.drive_diameter = float(driver_diameter)
				
		self.resetPosition()
				
		self.cov_twist=[0.003 , 0 , 0 , 0, 0, 0, #fake covariance
					   0 , 0.003 , 0 , 0, 0, 0,  #must get real covariances..
					   0 , 0 , 99999 , 0, 0, 0, 
					   0 , 0 , 0 , 99999, 0, 0, 
					   0 , 0 , 0 , 0, 99999, 0, 
					   0 , 0 , 0 , 0, 0, 0.001]
		
		self.cov_pos = [0.003 , 0 , 0 , 0, 0, 0, #fake covariance
					   0 , 0.003 , 0 , 0, 0, 0, 
					   0 , 0 , 99999 , 0, 0, 0, 
					   0 , 0 , 0 , 99999, 0, 0, 
					   0 , 0 , 0 , 0, 99999, 0, 
					   0 , 0 , 0 , 0, 0, 0.001]
		
	
	def update(self, l_encoder, r_encoder, dt):
		
		leftDist = l_encoder/ self.lw_encoder_per_meter
		rightDist = r_encoder/ self.rw_encoder_per_meter
		
		distance = (leftDist+ rightDist)/2.0
		dtheta = (rightDist - leftDist)/self.drive_diameter
		
		self.theta = dtheta +self.theta
		
		self.theta = self.theta%(2*math.pi)
		
		dX = math.cos(self.theta) * distance 
		dY = math.sin(self.theta) * distance	
		
		self.x = self.x+dX
		self.y = self.y +dY
		
		vel = distance/dt
		
		accel = (vel - self.accel)/dt
		
		i = last_vel_i
		
		self.last_vel[i] = vel
		self.last_accel[i] = accel
		self.vel = np.mean(self.last_vel)
		self.accel = np.mean(self.last_accel)
		
		if(i>= len(self.last_vel)):
			i =0
		else:
			i+=1
		
	def loadPositionCovariance(self, covariance):
		self.cov_pose = covariance
	def loadTwistCovariance(self, covariance):
		self.cov_twist = covariance
	
	def resetPosition(self):
		self.x, self.y = 0.0,0.0		
		self.vel , self.accel = 0.0,0.0
		
		self.last_vel = np.ndarray(4); self.last_vel_i =0
		self.last_accel = np.ndarray(4); self.last_vel_i=0

		
		self.theta  =0.0 
		self.vel_theta=0.0
		self.accel_theta=0.0
								
	def getEncoderCmd(self, cmd_vel)
	"""
		returns a touple for encoder speed
		for each 
	"""
		
	
		return left_vel , right _vel

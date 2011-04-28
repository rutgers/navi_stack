import numpy as np
import math

"""
References:
http://planning.cs.uiuc.edu/node659.html

"""

class DiffDriveModel:
	"""
	Model of a Differential drive robot
	"""
	def __init__(self, drive_diameter, lw_encoder_per_meter, rw_encoder_per_meter):
		self.lw_encoder_per_meter= float(lw_encoder_per_meter)
		self.rw_encoder_per_meter= float(rw_encoder_per_meter)
		
		self.drive_diameter = float(drive_diameter)
				
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
	def __str__(self):
		s= "Diff Odom: DriveDiameter-  %f   LeftEnc -  %f   RightEnc %f"
		return s%(self.drive_diameter, self.lw_encoder_per_meter, self.rw_encoder_per_meter) 
		
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
		
		accel = (vel - self.vel)/dt
		
		i = self.last_vel_i
		
		self.last_vel[i] = vel
		self.last_accel[i] = accel
		self.last_dtheta[i] = dtheta

		self.vel = np.mean(self.last_vel)
		self.accel = np.mean(self.last_accel)
		self.dtheta = np.mean(self.last_dtheta)
		
		if (self.last_vel_i > 10):
			self.last_vel_i =0
		
	def loadPositionCovariance(self, covariance):
		self.cov_pose = covariance
	def loadTwistCovariance(self, covariance):
		self.cov_twist = covariance
	
	def resetPosition(self):

		self.x, self.y = 0.0,0.0	
		
		#the vel/accel is smoothed out
		self.last_vel_i =0
		self.last_vel = np.array([0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0])
		self.last_accel = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
		self.last_dtheta = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

		self.vel = 0.0
		self.accel = 0.0	
		self.dtheta = 0.0
		
		self.theta  =0.0 
		self.vel_theta=0.0
		self.accel_theta=0.0
								
	def getEncoderCmd(self, forward_meter_per_sec, rot_rad_per_sec):
		"""
			returns a touple for encoder speed
			for each 
		"""
		velx = forward_meter_per_sec
		velt = rot_rad_per_sec
		
		left_motor = velx
		right_motor = velx
		
		#Compute the linear velocity 
		rot_lin_vel_wheel = velt*self.drive_diameter/2.0
		left_motor -= rot_lin_vel_wheel
		right_motor += rot_lin_vel_wheel
	
		left_motor *= self.lw_encoder_per_meter
		right_motor *= self.rw_encoder_per_meter
		
		return left_motor, right_motor
	
	def position(self):
		return self.x,self.y, 0, self.theta


def model_unit_test():
	m = DiffDriveModel(1,100,100)
	m.update(1,100,100);
	
	if (1.0,0.0,0.0) != m.position():
		s=  " (1.0,0.0,0.0) != m.position() after m.update(1,100,100)"
		s+= "\n it is %f %f %f"%m.position()
	
	lw,rw = m.getEncoderCmd(1,1)
	m.update(lw*.05,rw*.05, 0.05)
	

	

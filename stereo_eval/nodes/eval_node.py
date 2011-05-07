#!/usr/bin/python

from __future__ import print_function

import roslib
roslib.load_manifest('stereo_eval')

import rospy
import cv
import cv_bridge
import image_geometry
import message_filters
import geometry_msgs.msg
import sensor_msgs.msg
import stereo_msgs.msg

import csv
import math
import numpy
import sys

mf = message_filters

class EvalNode:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.model  = image_geometry.PinholeCameraModel()
		self.stereo = image_geometry.StereoCameraModel()

		self.board_rows = rospy.get_param('~board_rows', 7)
		self.board_cols = rospy.get_param('~board_cols', 7)
		self.board_size = rospy.get_param('~board_size', 0.10)
		self.border     = rospy.get_param('~border', 8)
		self.logfile    = rospy.get_param('~log', '/tmp/stereo_data.csv')

		self.pub_guess = rospy.Publisher('board_guess', geometry_msgs.msg.PoseStamped)
		self.pub_viz   = rospy.Publisher('board_image', sensor_msgs.msg.Image)

		# Synchronize monocular images with the corresponding stereo disparity map
		topic_cam1 = rospy.resolve_name('left')
		topic_cam2 = rospy.resolve_name('right')
		sub_img1  = mf.Subscriber(topic_cam1 + '/image_rect',  sensor_msgs.msg.Image)
		sub_info1 = mf.Subscriber(topic_cam1 + '/camera_info', sensor_msgs.msg.CameraInfo)
		sub_img2  = mf.Subscriber(topic_cam2 + '/image_rect',  sensor_msgs.msg.Image)
		sub_info2 = mf.Subscriber(topic_cam2 + '/camera_info', sensor_msgs.msg.CameraInfo)
		sub_disp  = mf.Subscriber('disparity', stereo_msgs.msg.DisparityImage)

		self.sub      = [ sub_img1, sub_info1, sub_img2, sub_info2, sub_disp ]
		self.sub_sync = mf.TimeSynchronizer(self.sub, 10)
		self.sub_sync.registerCallback(self.UpdateImage)

		# Log data to a CSV file for analysis.
		self.file   = open(self.logfile, 'w')
		self.logger = csv.writer(self.file, delimiter=',')

	def GetCorners(self, mono, subpix = True):
		(ok, corners) = cv.FindChessboardCorners(mono, (self.board_cols, self.board_rows),
		                                           cv.CV_CALIB_CB_ADAPTIVE_THRESH
		                                         | cv.CV_CALIB_CB_NORMALIZE_IMAGE)
		                                        # | cv.CALIB_CB_FAST_CHECK)

		# Reject the detection if any corners are too close to the edge of the image.
		w, h = cv.GetSize(mono)
		if not all([ (self.border < x < (w - self.border)) and (self.border < y < (h - self.border)) for (x, y) in corners ]):
			ok = False

		if ok and subpix:
			corners = cv.FindCornerSubPix(mono, corners, (5, 5), (-1, -1),
			                              (cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1))
		return (ok, corners)

	def GetCoordinates(self):
		coords = [ ]
		for y in range(0, self.board_rows):
			for x in range(0, self.board_cols):
				coords.append( (x * self.board_size, y * self.board_size, 0) )
		return coords

	def FindTransformation(self, corners, model):
		n = len(corners)

		# Camera intrinsics used for chessboard localization.
		mat_cam = cv.fromarray(model.intrinsicMatrix())
		mat_dis = cv.fromarray(model.distortionCoeffs())
		cv.Reshape(mat_cam, 0, 3)

		# Corresponding two-dimensional and three-dimensional points.
		pts_2d = cv.CreateMat(n, 2, cv.CV_32FC1)
		pts_3d = cv.CreateMat(n, 3, cv.CV_32FC1)

		for i in range(0, n):
			(pts_2d[i, 0], pts_2d[i, 1]) = corners[i]
			pts_3d[i, 0] = math.floor(i / self.board_cols) * self.board_size
			pts_3d[i, 1] = math.floor(i % self.board_cols) * self.board_size
			pts_3d[i, 2] = 0.0

		# Solve for the transformation from the model frame to the camera frame.
		rmat = cv.CreateMat(3, 3, cv.CV_32FC1)
		rvec = cv.CreateMat(3, 1, cv.CV_32FC1)
		tvec = cv.CreateMat(3, 1, cv.CV_32FC1)
		cv.FindExtrinsicCameraParams2(pts_3d, pts_2d, mat_cam, mat_dis, rvec, tvec, False)
		cv.Rodrigues2(rvec, rmat)

		# Merge the rotation and translation into a single transformation matrix.
		tmat = cv.CreateMat(4, 4, cv.CV_32FC1)
		cv.SetIdentity(tmat)
		cv.Copy(rmat, tmat[0:3, 0:3])
		cv.Copy(tvec, tmat[0:3, 3:4])
		return numpy.asmatrix(tmat)

	def UpdateImage(self, msg_img1, msg_info1, msg_img2, msg_info2, msg_disp):
		stamp = msg_img1.header.stamp
		frame = msg_img1.header.frame_id

		img  = self.bridge.imgmsg_to_cv(msg_img1, 'mono8')
		disp = self.bridge.imgmsg_to_cv(msg_disp.image, '32FC1')
		self.model.fromCameraInfo(msg_info1)
		self.stereo.fromCameraInfo(msg_info1, msg_info2)

		# Convert the image to color for rendering the detected chessboard.
		self.viz  = cv.CreateMat(img.rows, img.cols, cv.CV_8UC3)
		self.mask = cv.CreateMat(img.rows, img.cols, cv.CV_8UC1)
		cv.CvtColor(img, self.viz, cv.CV_GRAY2BGR)
		ok, corners = self.GetCorners(img, True)

		if ok:
			# Rasterize the chessboard onto the image.
			polygon = [
				corners[0],
				corners[self.board_cols - 1],
				corners[self.board_cols *  self.board_rows - 1],
				corners[self.board_cols * (self.board_rows - 1)],
			]
			cv.Set(self.mask, 0)
			cv.FillConvexPoly(self.mask, polygon, (255,))
			cv.PolyLine(self.viz, [ polygon ], True, (0, 0, 255), 2)

			# Accumulate variance as a function of distance.
			mono  = numpy.linalg.inv(self.FindTransformation(corners, self.model))
			dists = list()

			for y in range(0, img.rows):
				for x in range(0, img.cols):
					if self.mask[y, x] and disp[y, x] > 0:
						d = self.stereo.getZ(disp[y, x])
						dists.append(d)

			dists = numpy.array(dists)
			self.logger.writerow( (mono[2, 3], numpy.mean(dists), numpy.var(dists)) )

		msg_viz = self.bridge.cv_to_imgmsg(self.viz, encoding='bgr8')
		self.pub_viz.publish(msg_viz)

def main():
	rospy.init_node('stereo_eval')
	node = EvalNode()
	rospy.spin()
	node.file.close()

if __name__ == "__main__":
    main()

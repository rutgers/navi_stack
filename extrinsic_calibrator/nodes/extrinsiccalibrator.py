#!/usr/bin/python

from __future__ import print_function

import roslib
roslib.load_manifest('extrinsic_calibrator')

import rospy
import cv
import cv_bridge
import image_geometry
import message_filters
import geometry_msgs.msg
import sensor_msgs.msg
import tf

import math
import numpy

mf = message_filters

class ExtrinsicNode:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.model1 = image_geometry.PinholeCameraModel()
		self.model2 = image_geometry.PinholeCameraModel()

		self.board_rows = rospy.get_param('~board_rows', 7)
		self.board_cols = rospy.get_param('~board_cols', 7)
		self.board_size = rospy.get_param('~board_size', 0.10)
		self.border     = rospy.get_param('~border', 8)
		self.fr_boards  = [
			rospy.get_param('~frame_board0',  '/board0'),
			rospy.get_param('~frame_board1',  '/board1')
		]
		self.fr_camera  = [
			rospy.get_param('~frame_camera0', '/board0'),
			rospy.get_param('~frame_camera1', '/board1')
		]

		self.pub_tf = tf.TransformBroadcaster()

		# Synchronized left and right camera frames with monocular calibration parameters.
		# TODO: Use the approximate synchronization algorithm.
		topic_cam1 = rospy.resolve_name('camera0')
		topic_cam2 = rospy.resolve_name('camera1')
		sub_img1   = mf.Subscriber(topic_cam1 + '/image_raw', sensor_msgs.msg.Image)
		sub_img2   = mf.Subscriber(topic_cam2 + '/image_raw', sensor_msgs.msg.Image)
		sub_info1  = mf.Subscriber(topic_cam1 + '/camera_info', sensor_msgs.msg.CameraInfo)
		sub_info2  = mf.Subscriber(topic_cam2 + '/camera_info', sensor_msgs.msg.CameraInfo)

		self.sub_sync = mf.TimeSynchronizer([ sub_img1, sub_info1, sub_img2, sub_info2 ], 10)
		self.sub_sync.registerCallback(self.UpdateImage)

		# Visualization of calibration.
		self.gui_name  = 'Extrinsic Calibration'
		self.gui_delay = 10
		self.gui_win   = cv.NamedWindow(self.gui_name)

	def GetCorners(self, mono, subpix = True):
		(ok, corners) = cv.FindChessboardCorners(mono, (self.board_cols, self.board_rows), cv.CV_CALIB_CB_ADAPTIVE_THRESH
		                                         | cv.CV_CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FAST_CHECK)

		# Reject the detection if any corners are too close to the edge of the image.
		w, h = cv.GetSize(mono)
		if not all([ (self.border < x < (w - self.border)) and (self.border < y < (h - self.border)) for (x, y) in corners ]):
			ok = False

		if ok and subpix:
			corners = cv.FindCornerSubPix(mono, corners, (5, 5), (-1, -1),
			                              (cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1))
		return (ok, corners)

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
		return tmat

	def TransformationToTransform(self, T):
		return (T[0:3, 3], tf.transformations.quaternion_from_matrix(T))

	def UpdateImage(self, msg_img1, msg_info1, msg_img2, msg_info2):
		stamp  = msg_img1.header.stamp
		frame1 = msg_img1.header.frame_id
		frame2 = msg_img2.header.frame_id

		img1 = self.bridge.imgmsg_to_cv(msg_img1, "mono8")
		img2 = self.bridge.imgmsg_to_cv(msg_img2, "mono8")
		self.model1.fromCameraInfo(msg_info1)
		self.model2.fromCameraInfo(msg_info2)

		# Empty visualization; information will be superimposed later.
		viz_rows = max(img1.rows, img2.rows)
		viz_cols = img1.cols + img2.cols

		self.viz = cv.CreateMat(viz_rows, viz_cols, cv.CV_8UC3)
		cv.Set(self.viz, 0)
		img1_bgr = self.viz[0:img1.rows, 0:img1.cols]
		img2_bgr = self.viz[0:img2.rows, img1.cols:(img1.cols + img2.cols)]
		cv.CvtColor(img1, img1_bgr, cv.CV_GRAY2BGR)
		cv.CvtColor(img2, img2_bgr, cv.CV_GRAY2BGR)

		# Find corresponding chessboard corners in the two images.
		ok1, corners1 = self.GetCorners(img1, True)
		ok2, corners2 = self.GetCorners(img2, True)

		if ok1 and ok2:
			# Verify that the chessboards have the same orientation.
			l0, l1 = (corners1[0],  corners2[0])
			r0, r1 = (corners1[-1], corners2[-1])
			x_same = (l0[0] < l1[0] and r0[0] < r1[0]) or (l0[0] > l1[0] and r0[0] > r1[0])
			y_same = (l0[1] < l1[1] and r0[1] < r1[1]) or (l0[1] > l1[1] and r0[1] > r1[1])

			if x_same and y_same:
				T_M1 = numpy.asmatrix(self.FindTransformation(corners1, self.model1))
				T_M2 = numpy.asmatrix(self.FindTransformation(corners2, self.model2))
				T_21 = T_M1 * numpy.linalg.inv(T_M2)
				T_12 = T_M2 * numpy.linalg.inv(T_M1)

				t_M1, R_M1 = self.TransformationToTransform(T_M1)
				t_M2, R_M2 = self.TransformationToTransform(T_M2)
				self.pub_tf.sendTransform(t_M1, R_M1, stamp, self.fr_boards[0], msg_img1.header.frame_id)
				self.pub_tf.sendTransform(t_M2, R_M2, stamp, self.fr_boards[1], msg_img2.header.frame_id)

				t_12, R_12 = self.TransformationToTransform(T_12)
				t_21, R_21 = self.TransformationToTransform(T_21)
				self.pub_tf.sendTransform(t_12, R_12, stamp, self.fr_cameras[0], msg_img2.header.frame_id)
				self.pub_tf.sendTransform(t_21, R_21, stamp, self.fr_cameras[1], msg_img1.header.frame_id)

				# TODO: Calculate reprojection error. Save the transform with minimum error.
				cv.DrawChessboardCorners(img1_bgr, (self.board_rows, self.board_cols), corners1, True)
				cv.DrawChessboardCorners(img2_bgr, (self.board_rows, self.board_cols), corners2, True)

				print(T_21)

		cv.ShowImage(self.gui_name, self.viz)
		cv.WaitKey(self.gui_delay)

def main():
	rospy.init_node('extrinsic_calibration')
	node = ExtrinsicNode()
	rospy.spin()

if __name__ == "__main__":
    main()

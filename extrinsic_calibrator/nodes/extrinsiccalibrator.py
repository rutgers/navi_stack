#!/usr/bin/python

import roslib
roslib.load_manifest('extrinsic_calibrator')

import rospy
import cv
import cv_bridge
import image_geometry
import message_filters
import sensor_msgs.msg
import sensor_msgs.srv

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

		# Synchronized left and right camera frames with monocular calibration parameters.
		# TODO: Use the approximate synchronization algorithm.
		topic_cam1 = rospy.resolve_name('camera0')
		topic_cam2 = rospy.resolve_name('camera1')
		sub_img1   = mf.Subscriber(topic_cam1 + '/image_raw', sensor_msgs.msg.Image)
		sub_img2   = mf.Subscriber(topic_cam2 + '/image_raw', sensor_msgs.msg.Image)
		sub_info1  = mf.Subscriber(topic_cam1 + '/camera_info', sensor_msgs.msg.CameraInfo)
		sub_info2  = mf.Subscriber(topic_cam2 + '/camera_info', sensor_msgs.msg.CameraInfo)

		sub_sync = mf.TimeSynchronizer([ sub_img1, sub_info1, sub_img2, sub_info2 ], 10)
		sub_sync.registerCallback(self.UpdateImage)

		# Visualization of calibration.
		self.gui_name  = 'Extrinsic Calibration'
		self.gui_delay = 1
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
		mat_cam = cv.CreateMatHeader(3, 3, cv.CV_64FC1)
		mat_dis = cv.CreateMatHeader(5, 1, cv.CV_64FC1)
		mat_cam.SetData(model.intrinsicMatrix())
		mat_dis.SetData(model.distortionCoeffs())

		# Corresponding two-dimensional and three-dimensional points.
		pts_2d = cv.CreateMat(n, 2, cv.CV_64FC1)
		pts_3d = cv.CreateMat(n, 3, cv.CV_64FC1)

		for i in range(0, n):
			(pts_2d[i, 0], pts_2d[i, 1]) = corners[i]
			pts_3d[i, 0] = floor(i / self.board_cols) * self.board_size
			pts_3d[i, 1] = floor(i % self.board_cols) * self.board_size
			pts_3d[i, 2] = 0.0

		# Solve for the transformation from the model frame to the camera frame.
		rmat = cv.CreateMat(3, 3, cv.CV_64FC1)
		rvec = cv.CreateMat(3, 1, cv.CV_64FC1)
		tvec = cv.CreateMat(3, 1, cv.CV_64FC1)
		cv.FindExtrinsicCameraParams2(pts_3d, pts_2d, mat_cam, mat_dis, rvec, tvec, False)
		cv.Rodrigues2(rvec, rmat)

		# Merge the rotation and translation into a single transformation matrix.
		tmat = cv.CreateMat(3, 4, cv.CV_64FC1)
		cv.Copy(rmat, tmat[0:3, 0:3])
		cv.Copy(tvec, tmat[0:3, 3:4])
		return tmat

	def UpdateImage(self, msg_img1, msg_info1, msg_img2, msg_info2):
		img1 = self.bridge.imgmsg_to_cv(msg_img1, "mono8")
		img2 = self.bridge.imgmsg_to_cv(msg_img2, "mono8")
		self.model1.fromCameraInfo(msg_info1)
		self.model2.fromCameraInfo(msg_info2)

		ok1, corners1 = self.GetCorners(img1, True)
		ok2, corners2 = self.GetCorners(img2, True)

		print (ok1, ok2)

		if not ok1 or not ok2:
			return

		# Verify that the chessboards have the same orientation.
		l0, l1 = (corners1[0], corners1[-1])
		r0, r1 = (corners2[0], corners2[-1])
		x_same = (l0[0] < l1[0] and r0[0] < r1[0]) or (l0[0] > l1[0] and r0[0] > r1[0])
		y_same = (l0[1] < l1[1] and r0[1] < r1[1]) or (l0[1] > l1[1] and r0[1] > r1[1])

		if not x_same or not y_same:
			return

		T_1M = self.FindTransformation(corners1, self.model1)
		T_2M = self.FindTransformation(corners2, self.model2)
		T_M2 = cv.CreateMat(3, 4, cv.CVFC1)
		cv.Invert(T_2M, T_M2, cv.CV_LU)
		T_12 = T_1M * T_M2

		# TODO: Calculate reprojection error. Save the transform with minimum error.

		# Visualize the detected chessboards.
		# TODO: Overlay the reprojection error.
		# TODO: Overlay the center of the reprojected chessboard.
		viz_rows = max(msg_img1.rows, msg_img2.rows)
		viz_cols = msg_img1.cols + msg_img2.cols
		viz_bgr  = cv.CreateMat(viz_rows, viz_cols, cv.CV_8UC1)

		img1_bgr = cv.CreateMat(img1.rows, img1.cols, cv.CV_8UC3)
		img2_bgr = cv.CreateMat(img2.rows, img2.cols, cv.CV_8UC3)
		cv.cvtColor(img1, img1_bgr, cv.CV_GRAY2BGR)
		cv.cvtColor(img2, img2_bgr, cv.CV_GRAY2BGR)

		cv.DrawChessboardCorners(img1_bgr, (self.board_rows, self.board_cols), corners1, True)
		cv.DrawChessboardCorners(img2_bgr, (self.board_rows, self.board_cols), corners2, True)

		viz_bgr.Set(viz, 0)
		cv.Copy(img1_bgr, viz_bgr[0:img1.rows, 0:img1.cols])
		cv.Copy(img2_bgr, viz_bgr[0:img2.rows, (img1.cols + 1):(img1.cols + img2.cols + 1)])

		# Update the GUI.
		# TODO: Cleanly exit when ENTER or ESC is pressed.
		cv.ShowImage(self.gui_name, viz_bgr)
		cv.WaitKey(self.gui_delay)

def main():
	rospy.init_node('extrinsic_calibration')
	node = ExtrinsicNode()
	rospy.spin()

if __name__ == "__main__":
    main()

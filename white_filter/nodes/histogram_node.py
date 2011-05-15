#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('white_filter')

import cv_bridge
import rospy
import sensor_msgs.msg

import cv
import numpy
import sys


CH_RED   = 0
CH_GREEN = 1
CH_BLUE  = 2
CH_HUE   = 3
CH_SAT   = 4
CH_VAL   = 5
CH_LABEL = 6

class HistogramNode:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()

		# Histogram parameters.
		bins_hue = rospy.get_param('~bins_hue', 10)
		min_hue  = rospy.get_param('~min_hue', 0)
		max_hue  = rospy.get_param('~max_hue', 255)

		bins_sat = rospy.get_param('~bins_sat', 10)
		min_sat  = rospy.get_param('~min_sat', 0)
		max_sat  = rospy.get_param('~max_sat', 255)

		bins          = [ bins_hue, bins_sat ]
		ranges        = [ (min_hue, max_hue), (min_sat, max_sat) ]
		self.channels = [ CH_HUE, CH_SAT ]

		# Histogram matching parameters.
		width  = rospy.get_param('~window_width',  5)
		height = rospy.get_param('~window_height', 5)
		self.window = (width, height)
		self.method = cv.CV_COMP_INTERSECT

		# Load training data from a CSV file.
		train_path = rospy.get_param('~train_path')
		train_data = numpy.genfromtxt(train_path, delimiter=',', comments='@', dtype=float)

		# Build the HS-histogram of both positive and negative examples.
		pos = train_data[train_data[:,CH_LABEL] == 1, :]
		neg = train_data[train_data[:,CH_LABEL] == 0, :]
		self.hist_pos = self.LoadHistogram(pos, self.channels, bins, ranges, 1.0)
		self.hist_neg = self.LoadHistogram(neg, self.channels, bins, ranges, 1.0)

		val_min, val_max, loc_min, loc_max = cv.GetMinMaxHistValue(self.hist_pos)
		rospy.loginfo('positive histogram counts is [{0}, {1}]'.format(val_min, val_max))
		val_min, val_max, loc_min, loc_max = cv.GetMinMaxHistValue(self.hist_neg)
		rospy.loginfo('negative histogram counts is [{0}, {1}]'.format(val_min, val_max))

		self.sub = rospy.Subscriber('image', sensor_msgs.msg.Image, self.ImageCallback)
		self.pub = rospy.Publisher('white', sensor_msgs.msg.Image)

	def LoadHistogram(self, samples, channels, bins, ranges, factor):
		ch_numpy = [ samples[:, ch:(ch + 1)] / 255.0 for ch in channels ]
		ch_cvmat = map(cv.fromarray, ch_numpy)
		ch_image = map(cv.GetImage,  ch_cvmat)

		histogram = cv.CreateHist(bins, cv.CV_HIST_ARRAY, ranges, True)
		cv.CalcHist(ch_image, histogram, False)
		print(histogram)
		cv.NormalizeHist(histogram, factor)
		return histogram

	def TransformImage(self, bgr, hsv, channels, size, method):
		assert(bgr.type == cv.CV_32FC3)
		assert(hsv.type == cv.CV_32FC3)

		chs_all = [ cv.CreateMat(hsv.rows, hsv.cols, cv.CV_32FC1) for i in range(0, 6) ]
		cv.Split(bgr, chs_all[CH_BLUE], chs_all[CH_GREEN], chs_all[CH_RED], None)
		cv.Split(hsv, chs_all[CH_HUE],  chs_all[CH_SAT],   chs_all[CH_VAL], None)
		chs = [ cv.GetImage(chs_all[ch]) for ch in channels ]

		dst_pos = cv.CreateMat(hsv.rows - size[1] + 1, hsv.cols - size[0] + 1, cv.CV_32FC1)
		dst_neg = cv.CreateMat(hsv.rows - size[1] + 1, hsv.cols - size[0] + 1, cv.CV_32FC1)
		cv.CalcBackProjectPatch(chs, dst_pos, size, self.hist_pos, method, 1.0)
		cv.CalcBackProjectPatch(chs, dst_neg, size, self.hist_neg, method, 1.0)


		return (dst_pos, dst_neg)

	def ImageCallback(self, msg_img):
		# Convert the ROS message to a floating-point image in HSV-space.
		bgr_8u  = self.bridge.imgmsg_to_cv(msg_img, 'bgr8')
		hsv_8u  = cv.CreateMat(bgr_8u.rows, bgr_8u.cols, cv.CV_8UC3)
		bgr_32f = cv.CreateMat(bgr_8u.rows, bgr_8u.cols, cv.CV_32FC3)
		hsv_32f = cv.CreateMat(bgr_8u.rows, bgr_8u.cols, cv.CV_32FC3)

		cv.CvtColor(bgr_8u, hsv_8u, cv.CV_BGR2HSV)
		cv.ConvertScale(bgr_8u, bgr_32f, 1.0 / 255)
		cv.ConvertScale(hsv_8u, hsv_32f, 1.0 / 255)

		# Do the color space transformation.
		dst_pos, dst_neg = self.TransformImage(bgr_32f, hsv_32f, self.channels, self.window, self.method)

		# DEBUG: Normalization for rendering.
		val_min, val_max, loc_min, loc_max = cv.MinMaxLoc(dst_pos)
		print('min = {0}, max = {1}'.format(val_min, val_max))
		dst_32f = (dst_pos - val_min) / (val_max - val_min)
		dst_8u  = cv.CreateMat(dst_pos.rows, dst_pos.cols, cv.CV_8UC1)
		cv.ConvertScale(dst_32f, dst_8u, 255.0)

		# Publish the positive response; ignore the negative response.
		msg_out = self.bridge.cv_to_imgmsg(dst_8u, 'mono8')
		msg_out.header.stamp    = msg_img.header.stamp
		msg_out.header.frame_id = msg_img.header.frame_id
		self.pub.publish(msg_out)

def main(argv):
	rospy.init_node('histogram_node')
	node = HistogramNode()
	rospy.spin()

if __name__ == '__main__':
	sys.exit(main(sys.argv))

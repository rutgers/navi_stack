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


CHAN_RED   = 0
CHAN_GREEN = 1
CHAN_BLUE  = 2
CHAN_HUE   = 3
CHAN_SAT   = 4
CHAN_VAL   = 5
CHAN_LABEL = 6

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
		self.channels = [ CHAN_HUE, CHAN_SAT ]

		# Histogram matching parameters.
		width  = rospy.get_param('~window_width',  5)
		height = rospy.get_param('~window_height', 5)
		self.window = (width, height)
		self.method = cv.CV_COMP_INTERSECT

		# Load training data from a CSV file.
		train_path = rospy.get_param('~train_path')
		train_data = numpy.genfromtxt(train_path, delimiter=',', comments='@', dtype=float)

		# Build the HS-histogram of both positive and negative examples.
		pos = train_data[train_data[:,CHAN_LABEL] == 1, :]
		neg = train_data[train_data[:,CHAN_LABEL] == 0, :]
		self.hist_pos = self.LoadHistogram(pos, self.channels, bins, ranges, False)
		self.hist_neg = self.LoadHistogram(neg, self.channels, bins, ranges, False)

		self.sub = rospy.Subscriber('image', sensor_msgs.msg.Image, self.ImageCallback)
		self.pub = rospy.Publisher('white', sensor_msgs.msg.Image)

	def LoadHistogram(self, samples, channels, bins, ranges, normalize = False):
		ch_numpy = [ samples[:, ch:(ch + 1)] for ch in channels ]
		ch_cvmat = map(cv.fromarray, ch_numpy)
		ch_image = map(cv.GetImage,  ch_cvmat)

		histogram = cv.CreateHist(bins, cv.CV_HIST_ARRAY, ranges, True)
		cv.CalcHist(ch_image, histogram, False)

		if normalize:
			cv.NormalizeHist(histogram, 1.0)
		return histogram

	def TransformImage(self, hsv, channels, size, method):
		assert(hsv.type == cv.CV_32FC3)

		chs_all = [ cv.CreateMat(hsv.rows, hsv.cols, cv.CV_32FC1) for i in range(0, 3) ]
		cv.Split(hsv, chs_all[0], chs_all[1], chs_all[2], None)
		chs = [ cv.GetImage(chs_all[ch]) for ch in channels ]

		dst_pos = cv.CreateMat(hsv.rows - size[1] + 1, hsv.cols - size[0] + 1, cv.CV_32FC1)
		dst_neg = cv.CreateMat(hsv.rows - size[1] + 1, hsv.cols - size[0] + 1, cv.CV_32FC1)
		cv.CalcBackProjectPatch(chs, dst_pos, size, self.hist_pos, method, 1.0)
		cv.CalcBackProjectPatch(chs, dst_neg, size, self.hist_neg, method, 1.0)
		return (dst_pos, dst_neg)

	def ImageCallback(self, msg_img):
		# Convert the ROS message to a floating-point image in HSV-space.
		bgr_8u  = self.bridge.imgmsg_to_cv(msg_img, 'bgr8')
		bgr_32f = cv.CreateMat(bgr_8u.rows, bgr_8u.cols, cv.CV_32FC3)
		hsv_32f = cv.CreateMat(bgr_8u.rows, bgr_8u.cols, cv.CV_32FC3)

		cv.ConvertScale(bgr_8u, bgr_32f, 1.0 / 255)
		cv.CvtColor(bgr_32f, hsv_32f, cv.CV_BGR2HSV)

		# Do the color space transformation.
		dst_pos, dst_neg = self.TransformImage(hsv_32f, self.channels, self.window, self.method)

		# Publish the positive response; ignore the negative response.
		msg_out = self.bridge.cv_to_imgmsg(dst_pos, encoding='32FC1')
		msg_out.header.stamp    = msg_img.header.stamp
		msg_out.header.frame_id = msg_img.header.frame_id
		self.pub.publish(msg_out)

def main(argv):
	rospy.init_node('histogram_node')
	node = HistogramNode()
	rospy.spin()

if __name__ == '__main__':
	sys.exit(main(sys.argv))

#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('white_filter')

import cv
import csv
import itertools
import random
import sys

try:
	file_clr = sys.argv[1]
	file_tru = sys.argv[2]
	ker_size = int(sys.argv[3])
except ValueError:
	print('err: kernel width and height must be integers', file=sys.stderr)
except IndexError:
	print('err: incorrect number of arguments', file=sys.stderr)
	print('usage: label.py <image> <mask> <kernel size>', file=sys.stderr)
	sys.exit(1)

writer   = csv.writer(sys.stdout)
img_rgb = cv.LoadImageM(file_clr, cv.CV_LOAD_IMAGE_COLOR)
img_tru = cv.LoadImageM(file_tru, cv.CV_LOAD_IMAGE_GRAYSCALE)

if ker_size > 0:
	cv.Smooth(img_rgb, img_rgb, cv.CV_GAUSSIAN, ker_size, ker_size)

img_hsv = cv.CreateMat(img_rgb.rows, img_rgb.cols, cv.CV_8UC3)
cv.CvtColor(img_rgb, img_hsv, cv.CV_BGR2HSV)

assert(img_rgb.rows == img_tru.rows)
assert(img_rgb.cols == img_tru.cols)

# Build a list of all the data in-memory.
pixels = itertools.product(range(0, img_rgb.rows), range(0, img_rgb.cols))
labels = { 0: [ ], 1: [ ] }

for (y, x) in pixels:
	rgb   = map(int, img_rgb[y, x])
	hsv   = map(int, img_hsv[y, x])
	label = int(int(img_tru[y, x]) > 127)
	labels[label].append(rgb + hsv + [ label ])

# Force both sets to be of equal size.
k = min(map(len, labels.values()))
for (key, label) in labels.items():
	samples = random.sample(label, k)
	writer.writerows(samples)

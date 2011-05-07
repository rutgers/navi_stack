#! /usr/bin/env python

import roslib; roslib.load_manifest('stereo_webcam')
import cv

map_file = '/home/asher/ieee/navi_stack/navi_simulation/gazebo/Media/materials/textures/nav_field.jpg'



import time

if __name__ == '__main__':
	map_image = cv.LoadImageM(map_file,1 )
	cv.NamedWindow("map",1)
	
	scale = 1.0
	
	if (map_image.cols > 800):
		scale = map_image.cols/800.0
		dst = cv.CreateMat( int(map_image.rows / scale), int(map_image.cols / scale), cv.CV_8UC3)
		cv.Resize(map_image, dst)
		map_image = dst
	
	cv.ShowImage("map" , map_image)

	def mouse_cb(event, x, y, flags, param):
		if (event == cv.CV_EVENT_LBUTTONUP):
			print x*scale , y*scale
			pt = (x,y)
			cv.Circle(map_image, pt, int(38/scale), cv.Scalar(255,0,0), -1)
			cv.ShowImage("map" , map_image)
		pass
		
	cv.SetMouseCallback('map', mouse_cb)
	
	
	
	while cv.waitKey(25) != 27:
		time.sleep(0.025)
		pass
	cv.DestroyWindow('map')

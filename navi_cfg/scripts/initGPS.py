#! /usr/bin/env python

import sys
import os
import gpstime

import time
import serial


if __name__ == '__main__':
	
	if (len(sys.argv) >=2):
		print "initGPS.py port LAT LONG"
	
	set_latlong =(len(sys.argv) == 4)  
	
	port = sys.argv[1]
	
	gps = serial.Serial(port, 9600, timeout=0.06)
	
	t = time.gmtime()
	gtime= gpstime.gpsFromUTC(t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec)
	gweek, gwsec, gday, gdsec  = gtime
	
	time_cmd = 'SETAPPROXTIME %d %d'%(gweek, gwsec)
	print "sending :  ", time_cmd
	gps.write(time_cmd+'\n')
	
	
	
	
	

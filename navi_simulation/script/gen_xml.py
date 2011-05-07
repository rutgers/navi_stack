#! /usr/bin/env python


import sys


model = """
	<model:physical name="%s">
		<xyz>%d %d 0</xyz>
		<include embedded="true">
			<xi:include href="../objects/barrel_construction.model" />
		</include>
	</model:physical>
"""
model_name = 'barrel'

if __name__ == '__main__':
	pts_file = sys.argv[1]
	
	pts_f = open(pts_file, 'r')
	#model_f = open(model_file, 'r')
	
	scale = 0.02
	
	num = 0

	for line in pts_f:
		x,y = line.split()
		x = float(x)*scale  - 50
		y = -float(y)* scale +50
		
		print model%(model_name+str(num), x,y)
		num += 1
		
		

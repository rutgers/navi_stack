/*
 * laserPlaneFinder.h
 *
 *  Created on: May 4, 2011
 *      Author: asher
 */

#ifndef LASERPLANEFINDER_H_
#define LASERPLANEFINDER_H_

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>

// returns true if everything is found correclty
bool laserPlaneFinder(const sensor_msgs::LaserScan& scan, const geometry_msgs::Pose& approx_pos,
						const float& approx_plane_width, geometry_msgs::Pose& pose);


#endif /* LASERPLANEFINDER_H_ */

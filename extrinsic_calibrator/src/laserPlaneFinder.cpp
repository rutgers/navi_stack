/*
 * laserPlaneFinder.cpp
 *
 *  Created on: May 4, 2011
 *      Author: asher
 */



#include <extrinsic_calibrator/laserPlaneFinder.h>
#include <Eigen/Dense>
#include <math.h>
#include <LinearMath/btQuaternion.h>

#include <iostream>

bool laserPlaneFinder(const sensor_msgs::LaserScan& scan, const geometry_msgs::Pose& approx_pos,
						const float& approx_plane_width, geometry_msgs::Pose& pose){
	//figure out where in the scan the approximate position is

	int center_index=-1;

	//x and y are switched in atan2 because in this coordinate system, 0 angle
	// is at the x axiswhich is forward, rather than to the right
	double c_angle = atan2(approx_pos.position.y, approx_pos.position.x);
//	std::cout << "cangle is " << c_angle << std::endl;
	center_index = c_angle/ scan.angle_increment + (scan.ranges.size()-1)/2;

//	std::cout << "There are " << scan.ranges.size() << " pts \n";
	//std::cout << "The center index is " << center_index << std::endl;

	int left_edge_index =-1;
	int right_edge_index=-1;
	//move in each direction and watch for the sharp drop in range as we get to the edges of the board

	float last_range = scan.ranges[center_index];
	bool found_edge = false;
	for(int i=center_index; i >=0; i--){
		// detects the edge by saying that the difference in the range between a pt
		// on the plane and off the plane will be at least 1/5 of the plane width
		if (fabs(last_range - scan.ranges[i]) > approx_plane_width/5){
			right_edge_index = i;
			found_edge = true;
			break;
		}
		last_range = scan.ranges[i];
	}
	if (!found_edge) {
		//std::cout << "Fail on right edge\n";
		return false; //failed for find edge
	}
	found_edge = false;

	for(uint i=center_index; i <scan.ranges.size(); i++){
		// detects the edge by saying that the difference in the range between a pt
		// on the plane and off the plane will be at least 1/5 of the plane width
		if (fabs(last_range - scan.ranges[i]) > approx_plane_width/5){
			left_edge_index = i;
			found_edge = true;
			break;
		}
		last_range = scan.ranges[i];
	}
	if (!found_edge) {
		//std::cout << "Fail on left edge\n";
		return false; //failed to find edge
	}


	//std::cout << "Left index is " << left_edge_index << " right index is " << right_edge_index << std::endl;
	//each point is pushed on to
	int total_pts=  left_edge_index - right_edge_index-2;

	Eigen::VectorXd X(total_pts), Y(total_pts);
	for(int i=0; i < total_pts; i++){

		int offset= right_edge_index+i+1;
		X[i] = scan.ranges[offset] *cos(scan.angle_min+offset*scan.angle_increment);
		Y[i] = scan.ranges[offset] *sin(scan.angle_min+offset*scan.angle_increment);
		//std::cout << "X " << X[i] << "  Y  " << Y[i] << std::endl;

	}

	Eigen::Vector2d origin(X.mean(), Y.mean());

	//make them zero mean
	X.cwise() -=  origin[0];
	Y.cwise() -= origin[1];

	//solve for the theta
	//convert the slope to orientation
	double slope = (X.transpose()*X).inverse()*X.transpose()*Y;
	double theta = atan(slope);

	//pack checkerboard pose estimate
	geometry_msgs::Pose plane_pose;
	plane_pose.position.x = origin[0];
	plane_pose.position.y = origin[1];
	btQuaternion qt;
	qt.setEuler(theta,0,0);
	plane_pose.orientation.w = qt.getW();
	plane_pose.orientation.x = qt.getX();
	plane_pose.orientation.y = qt.getY();
	plane_pose.orientation.z = qt.getZ();

	pose =  plane_pose;
return true;
}

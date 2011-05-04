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



bool laserPlaneFinder(const sensor_msgs::LaserScan& scan, const geometry_msgs::Pose& approx_pos,
						const float& approx_plane_width, geometry_msgs::Pose& pose){
	//figure out where in the scan the approximate position is

	int center_index=-1;

	//x and y are switched in atan2 because in this coordinate system, 0 angle
	// is at the x axiswhich is forward, rather than to the right
	double c_angle = atan2(approx_pos.position.x, approx_pos.position.y);

	center_index = c_angle/ scan.angle_increment;

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
	}
	if (!found_edge) return false; //failed for find edge
	found_edge = false;

	for(uint i=center_index; i <=scan.ranges.size(); i++){
		// detects the edge by saying that the difference in the range between a pt
		// on the plane and off the plane will be at least 1/5 of the plane width
		if (fabs(last_range - scan.ranges[i]) > approx_plane_width/5){
			left_edge_index = i;
			found_edge = true;
			break;
		}
	}
	if (!found_edge) return false; //failed to find edge

	//each point is pushed on to
	int total_pts=  left_edge_index - right_edge_index;

	Eigen::VectorXd X(total_pts), Y(total_pts);
	for(int i=0; i < total_pts; i++){
		X[i] = scan.ranges[right_edge_index+i] *cos(scan.angle_min+i*scan.angle_increment);
		Y[i] = scan.ranges[right_edge_index+i] *sin(scan.angle_min+i*scan.angle_increment);
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

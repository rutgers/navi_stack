/*
 * laser_board_finder_node.cpp
 *
 *  Created on: May 4, 2011
 *      Author: asher
 */

#include <extrinsic_calibrator/laserPlaneFinder.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

using namespace geometry_msgs;
ros::Publisher g_pub;
geometry_msgs::Pose g_guess;
bool g_valid_guess = false;

void guess_cb(const geometry_msgs::Pose::ConstPtr msg){
	g_guess= *msg;
	g_valid_guess = true;
}

void scan_cb(const sensor_msgs::LaserScan::ConstPtr scan){
	if (! g_valid_guess) return;

	geometry_msgs::Pose p;
	if(laserPlaneFinder(*scan,g_guess,0.6096, p) ){
		g_pub.publish(p);
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "laser_board_finder");
	ros::NodeHandle n;
	g_pub = n.advertise<geometry_msgs::Pose> ("board_pose",1);
	ros::Subscriber sub = n.subscribe("board_guess", 1, guess_cb);
	ros::Subscriber sub_laser = n.subscribe("base_laser/scan",1, scan_cb);
	ros::spin();
}

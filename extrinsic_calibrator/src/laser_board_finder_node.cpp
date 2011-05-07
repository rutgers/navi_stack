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

#include <iostream>

using namespace geometry_msgs;
ros::Publisher g_pub;
geometry_msgs::PoseStamped g_guess;
bool g_valid_guess = false;

void guess_cb(const geometry_msgs::PoseStamped::ConstPtr msg){
	g_guess= *msg;
	g_valid_guess = true;
}

void scan_cb(const sensor_msgs::LaserScan::ConstPtr scan){
	if (! g_valid_guess) return;

	geometry_msgs::Pose p;
	std::cout << "About to search for the board\n";
	if(laserPlaneFinder(*scan,g_guess.pose,0.6096, p) ){
		std::cout << "Board has been found!\n";
		geometry_msgs::PoseStamped ps;
		std::cout << "The board is " << p <<std::endl;

		ps.header = scan->header;
		ps.pose = p;
		g_pub.publish(ps);
		g_guess.pose = p;
	}
	else std::cout << "finder fail!\n";

}


int main(int argc, char** argv){
	ros::init(argc, argv, "laser_board_finder");
	ros::NodeHandle n;
	g_pub = n.advertise<geometry_msgs::PoseStamped> ("board_pose",1);
	ros::Subscriber sub = n.subscribe("board_guess", 1, guess_cb);
	ros::Subscriber sub_laser = n.subscribe("base_laser/scan",1, scan_cb);
	g_guess.pose.position.x = 3.0;
	g_guess.pose.position.y = 0;
	g_valid_guess = true;
	ros::spin();
}

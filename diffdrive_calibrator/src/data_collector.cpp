/*
 * data_collector.cpp
 *
 *  Created on: May 31, 2011
 *      Author: asher
 */

#include <Eigen/Dense>
#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <navi_driver/Encoder.h>
#include <geometry_msgs/Pose2D.h>
#include <diffdrive_calibrator/CalibData.h>

#include <vector>
#include <string>

#include <math.h>

using namespace diffdrive_calibrator;


 class Listener{

private:

	CalibData data;

	bool started;
	geometry_msgs::Pose2D start_pose;
	geometry_msgs::Pose2D c_pose;


	ros::Subscriber sub_enc;
	ros::Subscriber sub_pose;

public:

	Listener(ros::NodeHandle* n){
		started = false;
		this->sub_enc = n->subscribe("drive/encoder", 1, &Listener::encoder_cb, this);
		this->sub_pose = n->subscribe("pose2D", 1, &Listener::pose_cb, this);

	}
	~Listener(){}


	void encoder_cb(const navi_driver::EncoderConstPtr& enc_msg){
		data.encoder.push_back(*(enc_msg));
	}

	void resetCollection(){
		data.encoder.clear();
		started = false;
	}

	void pose_cb(const geometry_msgs::Pose2DConstPtr& pose_msg){
		if (! started){
			started= true;
			this->start_pose = *pose_msg;
		}
		else{
			this->c_pose =*pose_msg;
		}
	}

	CalibData getData(){
		this->data.true_pose.x =  start_pose.x-c_pose.x;
		this->data.true_pose.y =  start_pose.y-c_pose.y;
		this->data.true_pose.theta =  start_pose.theta-c_pose.theta;

		this->data.true_pose_start = start_pose;
		this->data.true_pose_end = c_pose;
		return this->data;
	}
};




 int main(int argc, char** argv){
 	ros::init(argc,argv,"calib_data_collection");

 	/***** Collect the data  ****/
 	ros::NodeHandle n;
 	Listener lis(&n);

 	rosbag::Bag bag;
 	if (argc ==2 )
 	bag.open(argv[1], rosbag::bagmode::Write);
 	else  	bag.open("calib_data.bag", rosbag::bagmode::Append);

 	double data_duration =2;

 	for(uint v=0; v< 15; v++){
 		double start = ros::Time::now().toSec();
 		std::cout << "Start time is " << start << std::endl;
 		ros::Rate r(50);
 		while( (ros::Time::now().toSec()-start) < data_duration){
 			std::cout << "Ctime " << ros::Time::now() <<std::endl;
 			ros::spinOnce();
 			r.sleep();
 		}
 		std::cout << "One peice of data saved!\n";
 	bag.write("data", ros::Time::now(), lis.getData());
 	lis.resetCollection();
 	}
 	bag.close();
 }

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

#include <cmath>

#define PI 3.14159265


using namespace diffdrive_calibrator;



struct diff_model{
	float lw_encoder_per_meter;
	float rw_encoder_per_meter;
	float drive_diameter;
};

/*
 * Uses a diff model to update pose
 * pose is x y theta
 */
void modelUpdate(const diff_model& model, const int lw, const int rw, Eigen::Vector3d& pose){

	double ld = ( (double)lw) /model.lw_encoder_per_meter;
	double rd = ( (double )rw ) /model.rw_encoder_per_meter;
	double ddist = (ld+rd)/2;
	double dtheta = (rd-ld)/model.drive_diameter;

	//std::cout << "Update is " << ddist << "   " << dtheta <<std::endl;
	pose(0) += ddist* cos(pose(2));
	pose(1) += ddist * sin(pose(2));
	pose(2) += dtheta;
	//pose(2) = fmod(pose(2),2*PI);
}


void integrateModel(const diff_model& model,const  std::vector<navi_driver::Encoder>& enc_msgs, Eigen::Vector3d& pose){
	//std::cout << "Integrating model with " << enc_msgs.size() << " encoder msgs\n";
	for(uint i=0; i<enc_msgs.size(); i++){
		int left =enc_msgs[i].left;
		int right =enc_msgs[i].right;
		modelUpdate(model, left, right, pose);
		//std::cout << "The pose is " << pose.transpose() << std::endl;
	}
}

int computeErrors(const std::vector<CalibData>& data, const diff_model& model, Eigen::VectorXd& error){

	error.resize(data.size());
	int good=0;

	for (uint i=0; i<data.size(); i++){
		Eigen::Vector3d enc_pose;
		enc_pose << 0,0,0;

		integrateModel(model,data[i].encoder, enc_pose);
		Eigen::Vector3d true_pose;
		true_pose << -data[i].true_pose.x,  -data[i].true_pose.y, data[i].true_pose.theta;
	//	std::cout << "Encoder Pose : " << enc_pose.transpose() << std::endl;
	//	std::cout << "True Pose " << true_pose.transpose() << std::endl;

		Eigen::Vector3d e = (true_pose-enc_pose);
		Eigen::Vector2d xy; xy << e[0], e[1];
		if (enc_pose.norm() > 1){
			error(i) = e.mean(); //Error is simply the sum of the distance betwen poses
			good++;
		}
		else error(i)= 0;
	}

	return good;
}


void computeGradient( double diff_width, const std::vector<CalibData>& data, const diff_model & m,
					 Eigen::Vector3d& gradient, Eigen::VectorXd& c_error){

	diff_model c_m= m;
	Eigen::VectorXd f_error(data.size()), b_error(data.size());

	int valid_encs  = 	computeErrors(data,m,c_error);

	diff_width = 50;
	/*** Compute Derivate around lwE ***/
	c_m.lw_encoder_per_meter += diff_width;
	computeErrors(data,c_m, f_error);
	//std::cout << "LW Ferror :\n "<<f_error << std::endl;
	c_m.lw_encoder_per_meter -= 2*diff_width;
	computeErrors(data,c_m, b_error );
	//std::cout << "LW berror :\n "<<b_error << std::endl;

	std::cout << "The current error is " << c_error.transpose() << std::endl;
	gradient(0) = (f_error-b_error).sum()/valid_encs/(2*diff_width)*c_error.sum()/valid_encs;



	/*** Compute Derivative around rwE ***/
	c_m= m;
	c_m.rw_encoder_per_meter += diff_width;
	computeErrors(data,c_m, f_error);
	c_m.rw_encoder_per_meter -= 2*diff_width;
	computeErrors(data,c_m, b_error);

	computeErrors(data,m,c_error);
	gradient(1) = (f_error-b_error).sum()/valid_encs/(2*diff_width)*c_error.sum()/valid_encs;

	diff_width = 0.05;
	/*** Compute Derivative around dd ***/
	c_m= m;
	c_m.drive_diameter += diff_width;
	computeErrors(data,c_m, f_error);
	c_m.drive_diameter -= 2*diff_width;
	computeErrors(data,c_m, b_error);

	computeErrors(data,m,c_error);
	gradient(2) = (f_error-b_error).sum()/valid_encs/(2*diff_width)*c_error.sum()/valid_encs;
}

int main(int argc, char** argv){


	rosbag::Bag bag;
	bag.open("calib_data.bag", rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string("data"));

	std::vector<CalibData> data;

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	    {
	        CalibData::ConstPtr msg = m.instantiate<CalibData>();
	        if (msg != NULL)
	        	data.push_back(*msg);
	}
	bag.close();

	std::cout << "Data is loaded\n";


	//************** DEFINE MODEL GUESS **********/
	diff_model model;
	model.lw_encoder_per_meter = 975.315;
	model.rw_encoder_per_meter = 969.15;
	model.drive_diameter =  0.63;
/*
 * 970.717  rwE  974.625  DD 0.6685
 */


	Eigen::Vector3d gradient;
	Eigen::VectorXd c_error;
	int iters = 1000;
	for(int i =0; i < iters; i++){
		computeGradient(0.05,data,model,gradient,c_error);
		std::cout << "\nStep " << i << ": \n";
		std::cout << "The Model is  lwE " << model.lw_encoder_per_meter << "  rwE  " << model.rw_encoder_per_meter << "  DD "  << model.drive_diameter << std::endl;
		std::cout << "The average error is " << c_error.mean() << std::endl;
		std::cout << " The Gradient is :" << gradient.transpose() << "\n";
		double decay = exp(-((float)i)/((float)iters)/2);
		model.lw_encoder_per_meter = model.lw_encoder_per_meter - 10*decay*gradient(0);
		model.rw_encoder_per_meter = model.rw_encoder_per_meter -10*decay* gradient(1);
		model.drive_diameter= model.drive_diameter - decay*gradient(2);
	}

	std::cout << "\n\nThe final Model is  lwE " << model.lw_encoder_per_meter << "  rwE  " << model.rw_encoder_per_meter << "  DD "  << model.drive_diameter << std::endl;

/*
	Eigen::Vector3d p;
	p<< 0,0,0;
	modelUpdate(model, -950,950, p);
	modelUpdate(model, -950,950, p);
	modelUpdate(model, 950,950, p);
	std::cout << "The pose is " << p.transpose() << std::endl;
*/
}



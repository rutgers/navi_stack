#include <Eigen/Dense>
#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <navi_driver/Encoder.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>

#include <string>

#include <math.h>

#define PI 3.14159265


typedef struct data_entry{
	std::vector<navi_driver::Encoder> encoder;
	geometry_msgs::Pose2D true_pose;
} data_entry;


/*
void readData(std::string file, std::string enc_topic, std::string pose_topic, std::vector<navi_driver::EncoderConstPtr>& data_encoder,
								std::vector<geometry_msgs::Pose2d>& data_pose){

	rosbag::Bag bag;
	bag.open(file.c_str(), rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(enc_topic);
	topics.push_back(pose_topic);

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		if (m.getTopic() == enc_topic){
			data_encoder.push_back(m.instantiate<navi_driver::Encoder>() );
		}
		if (m.getTopic() == laser_topic) {
			data_laser.push_back(m.instantiate<geometry_msgs::Pose2D>());
		}
	}

	bag.close();
}

*/


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

	double ld = lw*model.lw_encoder_per_meter;
	double rd = rw *model.rw_encoder_per_meter;
	double ddist = (ld+rd)/2;
	double dtheta = (rd-ld)/model.drive_diameter;
	pose[0] += ddist* cos(pose[2]);
	pose[1] += ddist * sin(pose[2]);
	pose[2] += dtheta;
	pose[2] = fmod(pose[2],2*PI);
}


Eigen::Vector3d integrateModel(diff_model model,const  std::vector<navi_driver::Encoder>& enc_msgs){

	Eigen::Vector3d pose;
	for(int i=0; i<enc_msgs.size(); i++){
		int left =enc_msgs[i].left;
		int right =enc_msgs[i].right;
		modelUpdate(model, left, right, pose);
	}
	return pose;
}

Eigen::VectorXd computeErrors(const std::vector<data_entry>& data, const diff_model& model){

	Eigen::VectorXd error(data.size());

	for (uint i=0; i<data.size(); i++){
		Eigen::Vector3d enc_pose = integrateModel(model,data[i].encoder);
		Eigen::Vector3d true_pose;
		true_pose << data[i].true_pose.x,  data[i].true_pose.y, data[i].true_pose.theta;
		Eigen::Vector3d e = (true_pose-enc_pose);
		Eigen::Vector2d xy; xy << e[0], e[1];
		error(i) = xy.norm() + e[2]; //Error is simply the sum of the distance betwen poses
	}
}


void computeGradient( double diff_width, const std::vector<data_entry>& data, const diff_model & m,
					 Eigen::Vector3d& gradient){

	diff_model c_m= m;
	Eigen::VectorXd f_error(data.size()), b_error(data.size());

	/*** Compute Derivate around lwE ***/
	c_m.lw_encoder_per_meter += diff_width;
	f_error = computeErrors(data,c_m);
	c_m.lw_encoder_per_meter -= 2*diff_width;
	b_error = computeErrors(data,c_m);

	gradient(0) = (f_error-b_error).sum()/(2*diff_width)*computeErrors(data,m);



	/*** Compute Derivative around rwE ***/
	c_m= m;
	c_m.rw_encoder_per_meter += diff_width;
	f_error = computeErrors(data,c_m);
	c_m.rw_encoder_per_meter -= 2*diff_width;
	b_error = computeErrors(data,c_m);

	gradient(1) = (f_error-b_error).sum()/(2*diff_width)* computeErrors(data,m);

	/*** Compute Derivative around dd ***/
	c_m= m;
	c_m.drive_diameter += diff_width;
	f_error = computeErrors(data,c_m);
	c_m.drive_diameter -= 2*diff_width;
	b_error = computeErrors(data,c_m);

	gradient(2) = (f_error-b_error).sum()/(2*diff_width)*computeErrors(data,m);
}



 class Listener{

private:

	data_entry data;

	bool started;
	geometry_msgs::Pose2D start_pose;
	geometry_msgs::Pose2D c_pose;


	ros::Subscriber sub_enc;
	ros::Subscriber sub_pose;

public:

	Listener(ros::NodeHandle* n){
		started = false;
		this->sub_enc = n->subscribe("drive/encoder", 1, &Listener::encoder_cb, this);
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

	data_entry getData(){
		this->data.true_pose.x =  start_pose.x-c_pose.x;
		this->data.true_pose.y =  start_pose.y-c_pose.y;
		this->data.true_pose.theta =  start_pose.theta-c_pose.theta;
		return this->data;
	}
};


int main(int argc, char** argv){
	ros::init(argc,argv,"node");

	/***** Collect the data  ****/
	ros::NodeHandle n;
	Listener lis(&n);

	std::vector<data_entry> data;
	data.resize(10);
	double data_duration =2;

	for(uint i=0; i<data.size(); i++){
		ros::Time start = ros::Time::now();
		ros::Rate r(0.01);
		while( (ros::Time::now()-start).sec < data_duration){
			ros::spinOnce();
			r.sleep();
		}
	data[i]=lis.getData();
	lis.resetCollection();
	}


	/*** Perform   Calibration **********/
}

#include <fstream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>

#include "csv.hpp"
#include "hack_node.hpp"

namespace white_filter {

// nodelet conversion
HackNodelet::HackNodelet(void)
	: nh_priv("~")
{}

ros::NodeHandle &HackNodelet::getNodeHandle(void)
{
	return nh;
}

ros::NodeHandle &HackNodelet::getPrivateNodeHandle(void)
{
	return nh_priv;
}
// nodelet conversion

void HackNodelet::onInit(void)
{
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	nh.param<bool>("/gazebo", m_gazebo, false);
	if (m_gazebo) {
		ROS_WARN("using Gazebo-specific whitenes filter");
	}


	nh_priv.param<int>("threshold", m_threshold, -255);

	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	m_sub = m_it->subscribe("image", 1, &HackNodelet::Callback, this);
}

void HackNodelet::Callback(sensor_msgs::Image::ConstPtr const &msg_img)
{
	namespace enc = sensor_msgs::image_encodings;

	// Convert the message to the OpenCV datatype without copying it.
	cv::Mat src_8u;
	try {
		cv_bridge::CvImageConstPtr src_tmp = cv_bridge::toCvShare(msg_img, enc::BGR8);
		src_8u = src_tmp->image;
	} catch (cv_bridge::Exception const &e) {
		ROS_WARN_THROTTLE(10, "unable to parse image message");
		return;
	}

	cv::Mat dst_8u;
	if (m_gazebo) {
		cv::cvtColor(src_8u, dst_8u, CV_BGR2GRAY);
		cv::threshold(dst_8u, dst_8u, 150, 255, cv::THRESH_TOZERO);
	} else {
		// Intermediate results are in the range [-512,+512].
		cv::Mat src_16s;
		src_8u.convertTo(src_16s, CV_16SC3);

		// Hack color-space transformation that seemingly works. Don't know/care why.
		std::vector<cv::Mat> channels;
		cv::split(src_16s, channels);
		cv::Mat &b = channels[0];
		cv::Mat &g = channels[1];
		cv::Mat &r = channels[2];
		cv::Mat dst_16s = 2*b - g - r;

		// Back to 8-bit for publishing.
		dst_16s.convertTo(dst_8u, CV_8UC1, 0.50, 0);
	}

	// Convert the OpenCV data to an output message without copying.
	cv_bridge::CvImage msg_white;
	msg_white.header   = msg_img->header;
	msg_white.encoding = enc::MONO8;
	msg_white.image    = dst_8u;
	m_pub.publish(msg_white.toImageMsg());
}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hack_node");
	white_filter::HackNodelet node;
	node.onInit();
	ros::spin();
	return 0;
}

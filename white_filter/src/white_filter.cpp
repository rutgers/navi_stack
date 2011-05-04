#include "white_filter.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>

PLUGINLIB_DECLARE_CLASS(white_filter, white_nodelet, white_node::WhiteNodelet, nodelet::Nodelet)

namespace white_node {

void WhiteNodelet::onInit(void)
{
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	nh_priv.param<bool>("use_blue", m_use_blue, false);
	nh_priv.param<int>("threshold_val", m_threshold_val, 50);
	nh_priv.param<int>("threshold_sat", m_threshold_sat, 50);

	nh_priv.param<int>("blue_val", m_blue_val, 127);
	nh_priv.param<int>("blue_hue", m_blue_hue, 127);
	nh_priv.param<int>("blue_sat", m_blue_sat, 127);

	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	m_sub = m_it->subscribe("image", 1, &WhiteNodelet::Callback, this);
}

void WhiteNodelet::FilterWhite(cv::Mat src, cv::Mat &dst)
{
	// Convert the image into the HSV color space. By most ignoring pixel value,
	// this algorithm is made much more robust to changes in ambiant lighting.
	std::vector<cv::Mat> chans(3);
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV);
	cv::split(hsv, chans);
	cv::Mat &hue = chans[0];
	cv::Mat &sat = chans[1];
	cv::Mat &val = chans[2];

	// Remove extremely dark regions from the image. Since saturation is
	// arbitrary near black in the color space, these regions may have sufficiently
	// low saturation to be falsely detected as white.
	cv::Mat mask;
	cv::threshold(255 - sat, dst,  m_threshold_sat, 255, cv::THRESH_TOZERO);
	cv::threshold(val, mask, m_threshold_val, 255, cv::THRESH_BINARY);
	cv::min(mask, dst, dst);
}

void WhiteNodelet::FilterBlue(cv::Mat src, cv::Mat &dst)
{
	std::vector<cv::Mat> chans(3);
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV);
	cv::split(hsv, chans);
	cv::Mat &hue = chans[0];
	cv::Mat &sat = chans[1];
	cv::Mat &val = chans[2];

	cv::Mat good_sat;
	cv::Mat good_val;
	cv::threshold(sat, good_sat, m_blue_sat, 0, cv::THRESH_TOZERO);
	cv::threshold(val, good_val, m_blue_val, 0, cv::THRESH_TOZERO);

	cv::Mat good_hue = 255 - cv::abs(hue - m_blue_hue);

	cv::min(good_hue, good_sat, dst);
cv::min(good_val, dst,      dst);
}

void WhiteNodelet::Callback(sensor_msgs::Image::ConstPtr const &msg_img)
{
	namespace enc = sensor_msgs::image_encodings;

	cv::Mat src, dst;

	// Convert the message to the OpenCV datatype without copying it.
	try {
		cv_bridge::CvImageConstPtr src_tmp = cv_bridge::toCvShare(msg_img, enc::BGR8);
		src = src_tmp->image;
	} catch (cv_bridge::Exception const &e) {
		ROS_WARN_THROTTLE(10, "unable to parse image message");
		return;
	}

	if (m_use_blue) {
		FilterBlue(src, dst);
	} else {
		FilterWhite(src, dst);
	}

	// Convert the OpenCV data to an output message without copying.
	cv_bridge::CvImage msg_white;
	msg_white.header   = msg_img->header;
	msg_white.encoding = enc::MONO8;
	msg_white.image    = dst;
	m_pub.publish(msg_white.toImageMsg());
}

};

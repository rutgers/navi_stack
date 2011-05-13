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

	nh_priv.param<int>("threshold_sat",    m_threshold_sat,    127);
	nh_priv.param<int>("threshold_val",    m_threshold_val,    127);
	nh_priv.param<int>("threshold_hue_lo", m_threshold_hue_lo, 180);
	nh_priv.param<int>("threshold_hue_hi", m_threshold_hue_hi, 60);

	nh_priv.param<bool>("use_blue", m_use_blue, false);
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

	// Split the image into regions of low and high saturation.
	cv::Mat mask_lo, mask_hi;
	cv::threshold(sat, mask_lo, m_threshold_sat, 255, cv::THRESH_BINARY_INV);
	cv::threshold(sat, mask_hi, m_threshold_sat, 255, cv::THRESH_BINARY);

	// High saturation; remove hue up to the green range (~180).
	cv::Mat mask_hue_hi;
	cv::threshold(hue, mask_hue_hi, m_threshold_hue_hi, 255, cv::THRESH_BINARY);
	cv::min(mask_hue_hi, mask_hi, mask_hi);

	// Regions of low saturation; remove hue up to the yellow range (~60).
	cv::Mat mask_hue_lo;
	cv::threshold(hue, mask_hue_lo, m_threshold_hue_lo, 255, cv::THRESH_BINARY);
	cv::min(mask_hue_lo, mask_lo, mask_lo);

	// Eliminate low brightness where hue and saturation are arbitrary.
	cv::Mat mask_val;
	cv::threshold(val, mask_val, m_threshold_val, 255, cv::THRESH_BINARY);
	cv::min(mask_lo, mask_val, mask_lo);
	cv::min(mask_hi, mask_val, mask_hi);

	// Combine the search of the low and high saturation regions.
	cv::max(mask_lo, mask_hi, dst);
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

#include <fstream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>

#include "csv.hpp"
#include "hack_node.hpp"

#define HUE 0
#define SAT 1
#define VAL 2

namespace navi_white {

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

	nh_priv.param<bool>("debug", m_debug, false);
	nh_priv.param<int>("val_min",   m_val_min,   45);
	nh_priv.param<int>("hue_min",   m_hue_min,  130);
	nh_priv.param<int>("hue_max",   m_hue_max,  180);
	nh_priv.param<int>("sat_split", m_sat_split, 30);

	// Use dynamic_reconfigure to get all other parameters.
	m_srv_dr.setCallback(boost::bind(&HackNodelet::ReconfigureCallback, this, _1, _2));

	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	if (m_debug && m_gazebo) {
		ROS_WARN("debug topics are unsupported in Gazebo");
	}
	if (m_debug) {
		ROS_WARN("debug topics enabled; performance may be degraded");
		m_pub_hue = m_it->advertise("white_hue", 1);
		m_pub_sat = m_it->advertise("white_sat", 1);
		m_pub_val = m_it->advertise("white_val", 1);
		m_pub_hi  = m_it->advertise("white_hi", 1);
		m_pub_lo  = m_it->advertise("white_lo", 1);
	}
	m_sub = m_it->subscribe("image", 1, &HackNodelet::Callback, this);
}

void HackNodelet::ReconfigureCallback(NaviWhiteConfig &config, int32_t level)
{
	m_val_min   = config.val_min;
	m_hue_min   = config.hue_min;
	m_hue_max   = config.hue_max;
	m_sat_split = config.sat_split;
}

#if 0
void HackNodelet::cvtCYMK(cv::Mat src, cv::Mat &c, cv::Mat &y, cv::Mat &m, cv::Mat k)
{}
#endif

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

	// Remove noise in the grass by filtering.
	cv::Mat src_blur;
	cv::medianBlur(src_8u, src_blur, 5);

	cv::Mat dst_8u;
	if (m_gazebo) {
		cv::cvtColor(src_8u, dst_8u, CV_BGR2GRAY);
		cv::threshold(dst_8u, dst_8u, 150, 255, cv::THRESH_TOZERO);
	} else {
		cv::Mat hsv_8u;
		std::vector<cv::Mat> hsv_ch;
		cv::cvtColor(src_blur, hsv_8u, CV_BGR2HSV);
		cv::split(hsv_8u, hsv_ch);

		// Eliminate areas that have very low intensity since they have
		// arbitrary hue and saturation.
		cv::Mat mask_val;
		cv::threshold(hsv_ch[VAL], mask_val, m_val_min, 255, cv::THRESH_BINARY);

		// Hue does not change with illumination.
		// TODO: Need to binarize this
		cv::Mat mask_hue, mask_hue_min, mask_hue_max;
		cv::threshold(hsv_ch[HUE], mask_hue_min, m_hue_min, 255, cv::THRESH_BINARY);
		cv::threshold(hsv_ch[HUE], mask_hue_max, m_hue_max, 255, cv::THRESH_BINARY_INV);
		cv::min(mask_hue_min, mask_hue_max, mask_hue);

		// Split regions of low and high saturation.
		cv::Mat mask_sat_lo, mask_sat_hi;
		cv::threshold(hsv_ch[SAT], mask_sat_lo, m_sat_split, 255, cv::THRESH_BINARY_INV);
		cv::threshold(hsv_ch[SAT], mask_sat_hi, m_sat_split, 255, cv::THRESH_BINARY);

		// White looks different in direct sunlight and in shadows:
		//  1. sunlight: low saturation, high value
		//  2. shadow: high saturation, high hue
		cv::Mat mask_combo_hi;
		cv::min(mask_sat_hi, mask_hue, mask_combo_hi);
		cv::max(mask_sat_lo, mask_combo_hi, dst_8u);
		cv::min(mask_val, dst_8u, dst_8u);

		if (m_debug) {
			// Thresholded hue for the entire image.
			cv_bridge::CvImage msg_hue;
			msg_hue.header   = msg_img->header;
			msg_hue.encoding = enc::MONO8;
			msg_hue.image    = mask_hue;
			m_pub_hue.publish(msg_hue.toImageMsg());

			// Thresholded saturation for the entire image.
			cv_bridge::CvImage msg_sat;
			msg_sat.header   = msg_img->header;
			msg_sat.encoding = enc::MONO8;
			msg_sat.image    = mask_sat_lo;
			m_pub_sat.publish(msg_sat.toImageMsg());

			// Thresholded value for the entire image.
			cv_bridge::CvImage msg_val;
			msg_val.header   = msg_img->header;
			msg_val.encoding = enc::MONO8;
			msg_val.image    = mask_val;
			m_pub_val.publish(msg_val.toImageMsg());

			// Output from regions of high saturation.
			cv_bridge::CvImage msg_hi;
			msg_hi.header   = msg_img->header;
			msg_hi.encoding = enc::MONO8;
			msg_hi.image    = mask_combo_hi;
			m_pub_hi.publish(msg_hi.toImageMsg());

			// Output from regions of low saturation.
			cv_bridge::CvImage msg_lo;
			msg_lo.header   = msg_img->header;
			msg_lo.encoding = enc::MONO8;
			msg_lo.image    = mask_sat_lo;
			m_pub_lo.publish(msg_lo.toImageMsg());
		}
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
	navi_white::HackNodelet node;
	node.onInit();
	ros::spin();
	return 0;
}

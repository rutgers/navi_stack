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
	// Use dynamic_reconfigure to get all other parameters.
	m_srv_dr.setCallback(boost::bind(&HackNodelet::ReconfigureCallback, this, _1, _2));

	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	if (m_debug && m_gazebo) {
		ROS_WARN("debug topics are unsupported in Gazebo");
	}
	if (m_debug) {
		ROS_WARN("debug topics enabled; performance may be degraded");
		m_pub_blur         = m_it->advertise("white_blur",         1);
		m_pub_split        = m_it->advertise("white_split",        1);
		m_pub_shadow       = m_it->advertise("white_shadow",       1);
		m_pub_shadow_hue   = m_it->advertise("white_shadow_hue",   1);
		m_pub_shadow_val   = m_it->advertise("white_shadow_val",   1);
		m_pub_sunlight     = m_it->advertise("white_sunlight",     1);
		m_pub_sunlight_hue = m_it->advertise("white_sunlight_hue", 1);
		m_pub_sunlight_val = m_it->advertise("white_sunlight_val", 1);
	}
	m_sub = m_it->subscribe("image", 1, &HackNodelet::Callback, this);
}

void HackNodelet::ReconfigureCallback(NaviWhiteConfig &config, int32_t level)
{
	m_sat_split    = config.sat_split;
	m_shadow_hue   = config.shadow_hue;
	m_shadow_val   = config.shadow_val;
	m_sunlight_hue = config.sunlight_hue;
	m_sunlight_val = config.sunlight_val;
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

		cv::Mat shadow_sat, sunlight_sat;
		cv::threshold(hsv_ch[SAT], sunlight_sat, m_sat_split + 1, 255, cv::THRESH_BINARY_INV);
		cv::threshold(hsv_ch[SAT], shadow_sat,   m_sat_split + 0, 255, cv::THRESH_BINARY);

		cv::Mat shadow, shadow_hue, shadow_val;
		cv::threshold(hsv_ch[HUE], shadow_hue, m_shadow_hue, 255, cv::THRESH_BINARY);
		cv::threshold(hsv_ch[VAL], shadow_val, m_shadow_val, 255, cv::THRESH_BINARY);
		cv::min(shadow_hue, shadow_val, shadow);
		cv::min(shadow_sat, shadow,     shadow);

		cv::Mat sunlight, sunlight_hue, sunlight_val;
		cv::threshold(hsv_ch[HUE], sunlight_hue, m_sunlight_hue, 255, cv::THRESH_BINARY);
		cv::threshold(hsv_ch[VAL], sunlight_val, m_sunlight_val, 255, cv::THRESH_BINARY);
		cv::min(sunlight_hue, sunlight_val, sunlight);
		cv::min(sunlight_sat, sunlight,     sunlight);

		cv::max(shadow, sunlight, dst_8u);

		if (m_debug) {
			cv_bridge::CvImage msg_blur;
			msg_blur.header   = msg_img->header;
			msg_blur.encoding = enc::MONO8;
			msg_blur.image    = src_blur;
			m_pub_blur.publish(msg_blur.toImageMsg());

			cv_bridge::CvImage msg_split;
			msg_split.header   = msg_img->header;
			msg_split.encoding = enc::MONO8;
			msg_split.image    = shadow_sat;
			m_pub_split.publish(msg_split.toImageMsg());

			// Shadow
			cv_bridge::CvImage msg_shadow_hue;
			msg_shadow_hue.header   = msg_img->header;
			msg_shadow_hue.encoding = enc::MONO8;
			msg_shadow_hue.image    = shadow_hue;
			m_pub_shadow_hue.publish(msg_shadow_hue.toImageMsg());

			cv_bridge::CvImage msg_shadow_val;
			msg_shadow_val.header   = msg_img->header;
			msg_shadow_val.encoding = enc::MONO8;
			msg_shadow_val.image    = shadow_val;
			m_pub_shadow_val.publish(msg_shadow_val.toImageMsg());

			cv_bridge::CvImage msg_shadow;
			msg_shadow.header   = msg_img->header;
			msg_shadow.encoding = enc::MONO8;
			msg_shadow.image    = shadow;
			m_pub_shadow.publish(msg_shadow.toImageMsg());

			// Sunlight
			cv_bridge::CvImage msg_sunlight_hue;
			msg_sunlight_hue.header   = msg_img->header;
			msg_sunlight_hue.encoding = enc::MONO8;
			msg_sunlight_hue.image    = sunlight_hue;
			m_pub_sunlight_hue.publish(msg_sunlight_hue.toImageMsg());

			cv_bridge::CvImage msg_sunlight_val;
			msg_sunlight_val.header   = msg_img->header;
			msg_sunlight_val.encoding = enc::MONO8;
			msg_sunlight_val.image    = sunlight_val;
			m_pub_sunlight_val.publish(msg_sunlight_val.toImageMsg());

			cv_bridge::CvImage msg_sunlight;
			msg_sunlight.header   = msg_img->header;
			msg_sunlight.encoding = enc::MONO8;
			msg_sunlight.image    = sunlight;
			m_pub_sunlight.publish(msg_sunlight.toImageMsg());
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

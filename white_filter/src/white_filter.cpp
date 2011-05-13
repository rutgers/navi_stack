#include "white_filter.hpp"

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

PLUGINLIB_DECLARE_CLASS(white_filter, white_nodelet, white_node::WhiteNodelet, nodelet::Nodelet)

namespace white_node {

void WhiteNodelet::onInit(void)
{
	std::cout << "INIT" << std::endl;

	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	std::string path, delim, truth;
	nh_priv.param<int>("knn", m_k, 1);
	nh_priv.param<std::string>("train_path",  path,  "");
	nh_priv.param<std::string>("train_delim", delim, ",");
	nh_priv.param<std::string>("train_true",  truth, "line-true");

	// Load training data from a CSV file.
	std::fstream stream(path.c_str(), std::fstream::in);
	cv::Mat features, labels;
	Parse(stream, features, labels, delim[0], truth);
	stream.close();
	ROS_INFO("loaded %d training points", features.rows);

	// Train the kNN classifier using the training data.
	CvMat old_features = features;
	CvMat old_labels   = labels;
	m_knn.train(&old_features, &old_labels, NULL, false, m_k);
	ROS_INFO("trained kNN classifier with k_max = %d", m_k);

	// Subscribers and publishers.
	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	m_sub = m_it->subscribe("image", 1, &WhiteNodelet::Callback, this);
}

void WhiteNodelet::FilterWhite(cv::Mat bgr, cv::Mat &dst)
{
	// RGB
	std::vector<cv::Mat> ch_bgr(3);
	cv::split(bgr, ch_bgr);
	cv::Mat &r = ch_bgr[2];
	cv::Mat &g = ch_bgr[1];
	cv::Mat &b = ch_bgr[0];

	// HSV
	std::vector<cv::Mat> ch_hsv(3);
	cv::Mat hsv;
	cv::cvtColor(bgr, hsv, CV_BGR2HSV);
	cv::split(hsv, ch_hsv);
	cv::Mat &h = ch_hsv[0];
	cv::Mat &s = ch_hsv[1];
	cv::Mat &v = ch_hsv[2];

	// Use kNN to classify each pixel.
	for (int y = 0; y < bgr.rows; ++y)
	for (int x = 0; x < bgr.cols; ++x) {
		cv::Mat feature(1, 6, CV_32FC1);
		float *data = (float *)feature.data;
		data[0] = r.at<uint8_t>(y, x);
		data[1] = g.at<uint8_t>(y, x);
		data[2] = b.at<uint8_t>(y, x);
		data[3] = h.at<uint8_t>(y, x);
		data[4] = s.at<uint8_t>(y, x);
		data[5] = v.at<uint8_t>(y, x);

		CvMat feature_old = feature;
		float predicted = m_knn.find_nearest(&feature_old, m_k);
		dst.at<uint8_t>(y, x) = 255 * !!predicted;
	}
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

	FilterWhite(src, dst);

	// Convert the OpenCV data to an output message without copying.
	cv_bridge::CvImage msg_white;
	msg_white.header   = msg_img->header;
	msg_white.encoding = enc::MONO8;
	msg_white.image    = dst;
	m_pub.publish(msg_white.toImageMsg());
}

};

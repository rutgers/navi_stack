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
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	std::string path, delim;
	nh_priv.param<int>("kernel_size", m_ker_size, 3);
	nh_priv.param<std::string>("train_path",  path,  "");
	nh_priv.param<std::string>("train_delim", delim, ",");

	// Load the PCA transformation from the parameter server.
	XmlRpc::XmlRpcValue transforms;
	nh_priv.getParam("transform", transforms);
	ROS_ASSERT(transforms.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(transforms.size() > 0);
	int n = transforms.size();

	for (int i = 0; i < n; ++i) {
		XmlRpc::XmlRpcValue transform = transforms[i];
		ROS_ASSERT(transform.getType() == XmlRpc::XmlRpcValue::TypeArray);
		ROS_ASSERT(transform.size() == 6);

		cv::Mat coefs(6, 1, CV_32FC1);
		float  *data = coefs.ptr<float>(0);

		for (int j = 0; j < transform.size(); ++j) {
			XmlRpc::XmlRpcValue coef = transform[j];
			ROS_ASSERT(coef.getType() == XmlRpc::XmlRpcValue::TypeDouble);
			data[j] = (float)static_cast<double>(coef);
		}
		m_transforms.push_back(coefs);
	}
	NODELET_INFO("loaded %d PCA dimensions from parameter server", (int)transforms.size());

	// Load the line properties (in PCA-space) from the parameter server.
	XmlRpc::XmlRpcValue center;
	nh_priv.getParam("center", center);
	ROS_ASSERT(center.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(center.size() == n);
	m_center.create(1, n, CV_32FC1);

	for (int i = 0; i < n; ++i) {
		XmlRpc::XmlRpcValue coord = center[i];
		ROS_ASSERT(coord.getType() == XmlRpc::XmlRpcValue::TypeDouble);

		float value = static_cast<double>(coord);
		m_center.at<float>(0, i) = value;
	}
	NODELET_INFO("loaded PCA cluster center from parameter server");

	// Load axis weights (in PCA-space) from the parameter server.
	XmlRpc::XmlRpcValue weights;
	nh_priv.getParam("weight", weights);
	ROS_ASSERT(weights.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(weights.size() == n);
	m_weight.resize(n);

	for (int i = 0; i < n; ++i) {
		XmlRpc::XmlRpcValue weight = weights[i];
		ROS_ASSERT(weight.getType() == XmlRpc::XmlRpcValue::TypeDouble);
		m_weight[i] = static_cast<double>(weight);
	}
	NODELET_INFO("loaded PCA cluster center from parameter server");

	// Subscribers and publishers.
	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	m_sub = m_it->subscribe("image", 1, &WhiteNodelet::Callback, this);
}

void WhiteNodelet::FilterWhite(cv::Mat bgr, cv::Mat &dst)
{
	int features = m_transforms.size();
	int rows = bgr.rows;
	int cols = bgr.cols;
	dst.create(rows, cols, CV_32FC1);

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

	for (int y = 0; y < rows; ++y)
	for (int x = 0; x < cols; ++x) {
		// Build the (RGB, HSV) feature vector.
		cv::Mat feature(1, 6, CV_32FC1);
		float *data = feature.ptr<float>(0);
		int i = 0;
		data[i++] = r.at<uint8_t>(y, x);
		data[i++] = g.at<uint8_t>(y, x);
		data[i++] = b.at<uint8_t>(y, x);
		data[i++] = h.at<uint8_t>(y, x);
		data[i++] = s.at<uint8_t>(y, x);
		data[i++] = v.at<uint8_t>(y, x);

		// Transform each pixel into the PCA-space.
		cv::Mat feature_pcl(1, features, CV_32FC1);
		float *data_pcl = feature_pcl.ptr<float>(0);

		for (size_t ch = 0; ch < m_transforms.size(); ++ch) {
			cv::Mat wrapped = feature * m_transforms[ch];
			data_pcl[ch] = wrapped.at<float>(0, 0);
		}

		// Find the distance in PCA-space to the target.
		float &distance = dst.at<float>(y, x);
		distance = 0.0;

		for (size_t ch = 0; ch < m_transforms.size(); ++ch) {
			distance += m_weight[ch] * -fabs(feature_pcl.at<float>(0, ch) - m_center.at<float>(0, ch));
		}
	}

	cv::Mat mono8;
	cv::normalize(dst, mono8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	dst = mono8;

	dst = b;
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
		NODELET_WARN_THROTTLE(10, "unable to parse image message");
		return;
	}

	// Blur to reduce the visibility of individual blades of grass.
	cv::Mat src_blur;
	if (m_ker_size > 1) {
		cv::GaussianBlur(src, src_blur, cv::Size(m_ker_size, m_ker_size), 0.0);
	} else {
		src_blur = src;
	}
	FilterWhite(src_blur, dst);

	// Convert the OpenCV data to an output message without copying.
	cv_bridge::CvImage msg_white;
	msg_white.header   = msg_img->header;
	msg_white.encoding = enc::MONO8;
	msg_white.image    = dst;
	m_pub.publish(msg_white.toImageMsg());
}

};

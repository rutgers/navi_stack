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
#include "pca_nodelet.hpp"

namespace white_filter {

// nodelet conversion
PCANodelet::PCANodelet(void)
	: nh_priv("~")
{}

ros::NodeHandle &PCANodelet::getNodeHandle(void)
{
	return nh;
}

ros::NodeHandle &PCANodelet::getPrivateNodeHandle(void)
{
	return nh_priv;
}
// nodelet conversion

void PCANodelet::onInit(void)
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

		std::vector<float> coefs(6);
		for (int j = 0; j < transform.size(); ++j) {
			XmlRpc::XmlRpcValue coef = transform[j];
			ROS_ASSERT(coef.getType() == XmlRpc::XmlRpcValue::TypeDouble);
			coefs[j] = static_cast<double>(coef);
		}
		m_transforms.push_back(coefs);
	}
	ROS_INFO("loaded %d PCA dimensions from parameter server", (int)transforms.size());

	// Load the line properties (in PCA-space) from the parameter server.
	XmlRpc::XmlRpcValue center;
	nh_priv.getParam("center", center);
	ROS_ASSERT(center.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(center.size() == n);

	for (int i = 0; i < n; ++i) {
		XmlRpc::XmlRpcValue coord = center[i];
		ROS_ASSERT(coord.getType() == XmlRpc::XmlRpcValue::TypeDouble);
		m_center.push_back(static_cast<double>(coord));
	}

	// Load axis weights (in PCA-space) from the parameter server.
	XmlRpc::XmlRpcValue weights;
	nh_priv.getParam("weight", weights);
	ROS_ASSERT(weights.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(weights.size() == n);

	for (int i = 0; i < n; ++i) {
		XmlRpc::XmlRpcValue weight = weights[i];
		ROS_ASSERT(weight.getType() == XmlRpc::XmlRpcValue::TypeDouble);
		m_weight.push_back(static_cast<double>(weight));
	}

	// Threshold for regions of low intensity (where H and S are undefined).
	nh_priv.param<int>("value_min", m_min_value, 45.0);

	// Subscribers and publishers.
	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	m_sub = m_it->subscribe("image", 1, &PCANodelet::Callback, this);
}

void PCANodelet::FilterWhite(cv::Mat bgr_8u, cv::Mat &dst)
{
	int rows = bgr_8u.rows;
	int cols = bgr_8u.cols;

	cv::Mat bgr;
	bgr_8u.convertTo(bgr, CV_32FC3);

	// BGR-space, HSV-space
	std::vector<cv::Mat> ch_bgr(3), ch_hsv(3);
	cv::Mat hsv;
	cv::cvtColor(bgr, hsv, CV_BGR2HSV);
	cv::split(bgr, ch_bgr);
	cv::split(hsv, ch_hsv);

	// BGRHSV-space, the feature space
	std::vector<cv::Mat> features;
	features.insert(features.end(), ch_bgr.begin(), ch_bgr.end());
	features.insert(features.end(), ch_hsv.begin(), ch_hsv.end());

	// Find distances in the feature-space.
	dst.create(rows, cols, CV_32FC1);
	dst.setTo(0.0);

	for (size_t transform = 0; transform < m_transforms.size(); ++transform) {
		cv::Mat transformed(rows, cols, CV_32FC1, cv::Scalar(0.0));

		// Transform the image into the correct space.
		for (size_t feature = 0; feature < features.size(); ++feature) {
			transformed += m_transforms[transform][feature] * features[feature];
		}

		// Find the distance between each pixel and the cluster center.
		// TODO: Convert to SSD instead of SAD.
		dst += m_weight[transform] * cv::abs(transformed - m_center[transform]);
	}

	cv::Mat mono8;
	cv::normalize(dst, mono8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	dst = mono8;

	// Eliminate regions of low value where hue and sat are undefined.
	cv::Mat val_8u;
	ch_hsv[2].convertTo(val_8u, CV_8UC1);

	cv::Mat mask;
	cv::threshold(val_8u, mask, m_min_value, 255, cv::THRESH_BINARY);
	cv::min(dst, mask, dst);
}

void PCANodelet::Callback(sensor_msgs::Image::ConstPtr const &msg_img)
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pca_node");
	white_filter::PCANodelet node;
	node.onInit();
	ros::spin();
	return 0;
}

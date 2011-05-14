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
	nh_priv.param<std::string>("train_path",  path,  "");
	nh_priv.param<std::string>("train_delim", delim, ",");

	// Load training data from a CSV file.
	std::fstream stream(path.c_str(), std::fstream::in);
	cv::Mat features, labels;
	if (!stream.good()) {
		NODELET_ERROR("unable to open training data");
		return;
	}

	bool valid = Parse(stream, features, labels, delim[0]);
	stream.close();
	if (!valid) {
		NODELET_ERROR("unable to parse training data");
		return;
	}
	NODELET_INFO("loaded %d training points", features.rows);

	// Train the SVM using the training data.
	cv::SVMParams svm_params;
	svm_params.svm_type    = cv::SVM::C_SVC; // categorical
	svm_params.kernel_type = cv::SVM::RBF;   // radial basis function
	svm_params.term_crit.epsilon  = 1e-4;
	svm_params.term_crit.max_iter = 50;
	svm_params.term_crit.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

	CvMat old_features = features;
	CvMat old_labels   = labels;
	m_svm.train_auto(&old_features, &old_labels, NULL, NULL, svm_params);
	NODELET_INFO("trained SVM classifier classifier");

	// Subscribers and publishers.
	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	m_sub = m_it->subscribe("image", 1, &WhiteNodelet::Callback, this);
}

void WhiteNodelet::FilterWhite(cv::Mat bgr, cv::Mat &dst)
{
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

	// Use kNN to classify each pixel.
	for (int y = 0; y < rows; ++y)
	for (int x = 0; x < cols; ++x) {
		cv::Mat feature(1, 6, CV_32FC1);
		float *data = feature.ptr<float>(0);
		data[0] = r.at<uint8_t>(y, x);
		data[1] = g.at<uint8_t>(y, x);
		data[2] = b.at<uint8_t>(y, x);
		data[3] = h.at<uint8_t>(y, x);
		data[4] = s.at<uint8_t>(y, x);
		data[5] = v.at<uint8_t>(y, x);

		CvMat feature_old = feature;
		float pred = m_svm.predict(&feature_old);
		dst.at<uint8_t>(y, x) = 255 * !!pred;

#if 0
		std::cout << "(" << x << ", " << y << ") "
		          << "RGB(" << data[0] << ", "
		                    << data[1] << ", "
		                    << data[2] << ") "
		          << "HSV(" << data[3] << ", "
		                    << data[4] << ", "
		                    << data[5] << ") "
		          << "-> " << !!pred  << std::endl;
#endif
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
		NODELET_WARN_THROTTLE(10, "unable to parse image message");
		return;
	}

	NODELET_INFO("before");
	FilterWhite(src, dst);
	NODELET_INFO("after");


	// Convert the OpenCV data to an output message without copying.
	cv_bridge::CvImage msg_white;
	msg_white.header   = msg_img->header;
	msg_white.encoding = enc::MONO8;
	msg_white.image    = dst;
	m_pub.publish(msg_white.toImageMsg());
	std::cout << "publish" << std::endl;
}

};

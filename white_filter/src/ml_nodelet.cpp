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
#include "ml_nodelet.hpp"

PLUGINLIB_DECLARE_CLASS(white_filter, ml_nodelet, white_filter::MLNodelet, nodelet::Nodelet)

namespace white_filter {

void MLNodelet::onInit(void)
{
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	int n_max;
	double weight;
	std::string path, delim;
	nh_priv.param<int>("kernel_size", m_ker_size, 3);
	nh_priv.param<int>("train_size",  n_max, 2500);
	nh_priv.param<double>("svm_weight", weight, 0.5);
	nh_priv.param<std::string>("train_path",  path,  "");
	nh_priv.param<std::string>("train_delim", delim, ",");

	// Load training data from a CSV file.
	std::fstream stream(path.c_str(), std::ios::in);
	cv::Mat features, labels;
	if (!stream.good()) {
		NODELET_ERROR("unable to open training data");
		return;
	}

	bool valid = Parse(stream, features, labels, delim[0]);
	stream.close();
	if (!valid) {
		NODELET_ERROR("training data is invalid");
		return;
	}
	features = features / 255.0;
	NODELET_INFO("loaded %d training points", features.rows);

	// Limit the amount of training data so OpenCV doesn't crash.
	int n = features.rows;
	if (n > n_max){
		features = features(cv::Range(0, n_max), cv::Range::all());
		labels   = labels(cv::Range(0, n_max), cv::Range::all());
		NODELET_WARN("using %d of %d training points", n_max, n);
	}

	// Train the SVM.
	CvMat old_features = features;
	CvMat old_labels   = labels;
	CvSVMParams params;
	params.svm_type    = cv::SVM::C_SVC;
	params.kernel_type = cv::SVM::RBF;
	params.term_crit.epsilon  = 1e-2;
	params.term_crit.max_iter = 25;
	params.term_crit.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

	cv::Mat weights(1, 2, CV_32FC1);
	weights.at<float>(0, 0) = weight;
	weights.at<float>(0, 1) = 1.0 - weight;
	CvMat old_weights    = weights;
	params.class_weights = &old_weights;

	m_ml.train_auto(features, labels, cv::Mat(), cv::Mat(), params);
	NODELET_INFO("trained SVM with %d points and %d support vectors",
	             features.rows,
	             m_ml.get_support_vector_count()
	);

	// Subscribers and publishers.
	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	m_sub = m_it->subscribe("image", 1, &MLNodelet::Callback, this);
}

void MLNodelet::FilterWhite(cv::Mat bgr_8u, cv::Mat &dst)
{
	int rows = bgr_8u.rows;
	int cols = bgr_8u.cols;

	dst.create(rows * cols, 1, CV_32FC1);

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
	cv::Mat bgrhsv;
	features.insert(features.end(), ch_bgr.begin(), ch_bgr.end());
	features.insert(features.end(), ch_hsv.begin(), ch_hsv.end());
	cv::merge(features, bgrhsv);
	bgrhsv = bgrhsv.reshape(1, cols * rows);
	bgrhsv = bgrhsv / 255.0;

	// Classify each point.
	for (int i = 0; i < bgrhsv.rows; ++i) {
		CvMat  feature = bgrhsv.row(i);
		float &value   = dst.at<float>(i, 0);
		value = m_ml.predict(&feature);
	}

	cv::Mat mono8;
	dst = dst.reshape(1, rows);
	cv::normalize(dst, mono8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	dst = mono8;
}

void MLNodelet::Callback(sensor_msgs::Image::ConstPtr const &msg_img)
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

}

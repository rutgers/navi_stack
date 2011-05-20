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
#include "histogram_nodelet.hpp"

PLUGINLIB_DECLARE_CLASS(white_filter, histogram_nodelet, white_filter::HistogramNodelet, nodelet::Nodelet)

namespace white_filter {

void HistogramNodelet::onInit(void)
{
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	// Histogram parameters.
	nh_priv.param<int>("bins_hue", m_bins_hue, 10);
	nh_priv.param<int>("bins_sat", m_bins_sat, 10);
	nh_priv.param<int>("win_width",  m_win_width,  9);
	nh_priv.param<int>("win_height", m_win_height, 9);
	m_method = CV_COMP_INTERSECT;
	m_haystack = boost::make_shared<IntegralHistogram>(m_bins_hue, m_bins_sat);

	// Training data for target histogram.
	double weight;
	std::string path, delim;
	nh_priv.param<int>("kernel_size", m_ker_size, 3);
	nh_priv.param<double>("svm_weight", weight, 0.5);
	nh_priv.param<std::string>("train_path",  path,  "");
	nh_priv.param<std::string>("train_delim", delim, ",");

	// Load training data from a CSV file.
	std::fstream stream(path.c_str(), std::ios::in);
	cv::Mat features, labels;
	if (!stream.good()) {
		NODELET_ERROR("unable to open histogram data");
		return;
	}

	bool valid = Parse(stream, features, labels, delim[0]);
	stream.close();
	if (!valid) {
		NODELET_ERROR("histogram data is invalid");
		return;
	}
	NODELET_INFO("loaded histogram of %d points", features.rows);

	// TODO: Build the target histogram.
	BuildNeedleHistogram(features, m_needle);

	// Subscribers and publishers.
	m_it = boost::make_shared<image_transport::ImageTransport>(nh);
	m_pub = m_it->advertise("white", 1);
	m_sub = m_it->subscribe("image", 1, &HistogramNodelet::Callback, this);
}

void HistogramNodelet::Callback(sensor_msgs::Image::ConstPtr const &msg_img)
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

	// Use histogram matching to isolate the line.
	cv::Mat dst_32f, dst_8u;
	MatchNeedleHistogram(src, dst_32f);
	cv::normalize(dst_32f, dst_8u, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	// Convert the OpenCV data to an output message without copying.
	cv_bridge::CvImage msg_white;
	msg_white.header   = msg_img->header;
	msg_white.encoding = enc::MO3NO8;
	msg_white.image    = dst_8u;
	m_pub.publish(msg_white.toImageMsg());
}

void HistogramNodelet::BuildNeedleHistogram(cv::Mat data, cv::MatND &needle)
{
	// Convert from a [ n x 6 ] 1-channel matrix to a [ n x 1 ] 6-channel matrix.
	cv::Mat data_ch = data.reshape(data.cols, data.rows);

	cv::Mat arrays[] = { data_ch };
	int chans[] = { 3, 4 };                   // hue and sat channels (BGRHSV)
	int sizes[] = { m_bins_hue, m_bins_sat }; // bins per channel
	float const range[]   = { 0, 255 };       // 8-bit channel depth
	float const *ranges[] = { range, range };
	int dims = 2;
	cv::calcHist(arrays, 1, chans, cv::Mat(), needle, dims, sizes, ranges, true, false);
}

void HistogramNodelet::MatchNeedleHistogram(cv::Mat src, cv::Mat &dst)
{
	dst.create(src.rows, src.cols, CV_32FC1);
	dst.setTo(0.0);
	m_haystack->LoadImage(src);

	for (int y = m_win_height / 2; y < src.rows - m_win_height / 2; ++y)
	for (int x = m_win_width  / 2; x < src.cols - m_win_width  / 2; ++x) {
		cv::Rect patch(cv::Point2i(x, y), cv::Size(m_win_width, m_win_height));

		// TODO: Why is this exceeding the image dimensions?
		// TODO: Is this off-by-one?
		patch = patch & cv::Rect(0, 0, src.cols - 1, src.rows - 1);

		cv::MatND hist_patch;
		m_haystack->GetPatch(patch, hist_patch);
		dst.at<float>(y, x) = cv::compareHist(hist_patch, m_needle, m_method);
	}
}

}

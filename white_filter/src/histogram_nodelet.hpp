#ifndef ML_NODELET_HPP_
#define ML_NODELET_HPP_

#include <vector>
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

#include "integral_histogram.hpp"

namespace white_filter {

class HistogramNodelet : public nodelet::Nodelet {
public:
	virtual void onInit(void);
	void Callback(sensor_msgs::Image::ConstPtr const &ptr);

	void BuildNeedleHistogram(cv::Mat data, cv::MatND &needle);
	void MatchNeedleHistogram(cv::Mat src, cv::Mat &dst);

private:
	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Subscriber m_sub;
	image_transport::Publisher  m_pub;

	cv::MatND m_needle;
	boost::shared_ptr<IntegralHistogram> m_haystack;

	int m_bins_hue, m_bins_sat;
	int m_win_width, m_win_height;
	int m_ker_size;
	int m_method;
};
};
#endif

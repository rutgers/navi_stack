#ifndef ML_NODELET_HPP_
#define ML_NODELET_HPP_

#include <vector>
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

namespace white_filter {

class MLNodelet : public nodelet::Nodelet {
public:
	virtual void onInit(void);

	void Callback(sensor_msgs::Image::ConstPtr const &ptr);
	void FilterBlue(cv::Mat src, cv::Mat &dst);
	void FilterWhite(cv::Mat src, cv::Mat &dst);

private:
	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Subscriber m_sub;
	image_transport::Publisher  m_pub;

	cv::SVM m_ml;
	int m_ker_size;

	// for capstone demo using blue duck tape
	bool m_use_blue;
	int m_blue_hue;
	int m_blue_sat;
	int m_blue_val;
};
};
#endif

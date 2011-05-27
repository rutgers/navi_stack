#ifndef PCA_NODELET_HPP_
#define PCA_NODELET_HPP_

#include <vector>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace white_filter {

class PCANodelet {
public:
	// nodelet conversion
	PCANodelet(void);
	ros::NodeHandle &getNodeHandle(void);
	ros::NodeHandle &getPrivateNodeHandle(void);
	virtual void onInit(void);
	// nodelet conversion

	void Callback(sensor_msgs::Image::ConstPtr const &ptr);
	void FilterBlue(cv::Mat src, cv::Mat &dst);
	void FilterWhite(cv::Mat src, cv::Mat &dst);

private:
	ros::NodeHandle nh, nh_priv;

	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Subscriber m_sub;
	image_transport::Publisher  m_pub;

	std::vector<float> m_center;
	std::vector<float> m_weight;
	std::vector<std::vector<float> > m_transforms;
	int m_ker_size;

	// for capstone demo using blue duck tape
	bool m_use_blue;
	int m_blue_hue;
	int m_blue_sat;
	int m_blue_val;
};
};
#endif

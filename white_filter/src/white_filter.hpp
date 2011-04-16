#ifndef WHITE_FILTER_HPP_
#define WHITE_FILTER_HPP_

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

namespace white_node {

class WhiteNodelet : public nodelet::Nodelet {
public:
	virtual void onInit(void);

	void Callback(sensor_msgs::Image::ConstPtr const &ptr);

private:
	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Subscriber m_sub;
	image_transport::Publisher  m_pub;

	bool m_use_low;
	bool m_use_high;
	int m_threshold_hue;
	int m_threshold_sat;
};
};
#endif

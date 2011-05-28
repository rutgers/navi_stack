#ifndef HACK_NODE_HPP_
#define HACK_NODE_HPP_

#include <vector>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace white_filter {

class HackNodelet {
public:
	// nodelet conversion
	HackNodelet(void);
	ros::NodeHandle &getNodeHandle(void);
	ros::NodeHandle &getPrivateNodeHandle(void);
	virtual void onInit(void);
	// nodelet conversion

	void Callback(sensor_msgs::Image::ConstPtr const &ptr);

private:
	ros::NodeHandle nh, nh_priv;
	int m_threshold;

	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Subscriber m_sub;
	image_transport::Publisher  m_pub;
};
};
#endif

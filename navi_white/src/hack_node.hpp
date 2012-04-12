#ifndef HACK_NODE_HPP_
#define HACK_NODE_HPP_

#include <vector>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <navi_white/NaviWhiteConfig.h>

namespace navi_white {
namespace dr = dynamic_reconfigure;

class HackNodelet {
public:
	// nodelet conversion
	HackNodelet(void);
	ros::NodeHandle &getNodeHandle(void);
	ros::NodeHandle &getPrivateNodeHandle(void);
	virtual void onInit(void);
	// nodelet conversion

	void ReconfigureCallback(NaviWhiteConfig &config, int32_t level);
	void Callback(sensor_msgs::Image::ConstPtr const &ptr);

private:
	ros::NodeHandle nh, nh_priv;

	bool m_debug;
	bool m_gazebo;

	int m_sat_split;
	int m_shadow_hue,   m_shadow_val;
	int m_sunlight_hue, m_sunlight_val;


	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Subscriber m_sub;
	image_transport::Publisher  m_pub;
	image_transport::Publisher  m_pub_blur;
	image_transport::Publisher  m_pub_split;
	image_transport::Publisher  m_pub_shadow;
	image_transport::Publisher  m_pub_shadow_hue;
	image_transport::Publisher  m_pub_shadow_val;
	image_transport::Publisher  m_pub_sunlight;
	image_transport::Publisher  m_pub_sunlight_hue;
	image_transport::Publisher  m_pub_sunlight_val;
	dr::Server<NaviWhiteConfig> m_srv_dr;
};
};
#endif

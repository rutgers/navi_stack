#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>

namespace it = image_transport;

class WhiteFilterNode {
public:
	WhiteFilterNode(ros::NodeHandle nh, ros::NodeHandle nh_priv);
	void Callback(sensor_msgs::Image::ConstPtr const &ptr);

private:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_priv;

	it::ImageTransport m_it;
	it::Subscriber     m_sub_rgb;
	it::Publisher      m_pub_white;

	int m_threshold_hue;
	int m_threshold_sat;
};

WhiteFilterNode::WhiteFilterNode(ros::NodeHandle nh, ros::NodeHandle nh_priv)
	: m_nh(nh),
	  m_nh_priv(nh_priv),
	  m_it(nh)
{
	m_sub_rgb   = m_it.subscribe("image", 1, &WhiteFilterNode::Callback, this);
	m_pub_white = m_it.advertise("white", 1);

	nh_priv.param<int>("threshold_hue", m_threshold_hue, 180);
	nh_priv.param<int>("threshold_sat", m_threshold_sat, 127);
}

void WhiteFilterNode::Callback(sensor_msgs::Image::ConstPtr const &msg_img)
{
	namespace enc = sensor_msgs::image_encodings;

	// Convert the message to the OpenCV datatype without copying it.
	cv::Mat src;
	try {
		cv_bridge::CvImageConstPtr src_tmp = cv_bridge::toCvShare(msg_img, enc::BGR8);
		src = src_tmp->image;
	} catch (cv_bridge::Exception const &e) {
		ROS_WARN_THROTTLE(10, "unable to parse image message");
		return;
	}

	// Convert the image into the HSV color space. By most ignoring pixel value,
	// this algorithm is made much more robust to changes in ambiant lighting.
	std::vector<cv::Mat> chans(3);
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV);
	cv::split(hsv, chans);
	cv::Mat &hue = chans[0];
	cv::Mat &sat = chans[1];
	cv::Mat &val = chans[2];

	// Split the image into regions of low and high saturation. Pixels with a
	// low saturation can be assumed to be white, but those with high saturation
	// need additional processing.
	cv::Mat sat_lo, sat_hi;
	cv::threshold(sat, sat_lo, m_threshold_sat, 255, cv::THRESH_BINARY_INV);
	cv::threshold(sat, sat_hi, m_threshold_sat, 255, cv::THRESH_BINARY);

	// For pixels with high saturation, remove all those with hue below less
	// than blue/green. White pixels in a shadow generally have high hue, where
	// other colors in the image primarily have low hue.
	cv::Mat hue_hi, white_lo;
	cv::threshold(hue, hue_hi, m_threshold_hue, 255, cv::THRESH_BINARY);
	cv::min(sat_hi, hue_hi, white_lo);

	// Merge the results of the "low sat" and "high sat" filters.
	cv::Mat &white_hi = sat_lo;
	cv::Mat  white;
	cv::max(white_lo, white_hi, white);

	// Use this binary "white" indicator to mask the value image.
	cv::Mat whiteness;
	cv::min(val, white, whiteness);

	// Convert the OpenCV data to an output message without copying.
	cv_bridge::CvImage msg_white;
	msg_white.header   = msg_img->header;
	msg_white.encoding = enc::MONO8;
	msg_white.image    = whiteness;
	m_pub_white.publish(msg_white.toImageMsg());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "white_filter");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	WhiteFilterNode node(nh, nh_priv);
	ros::spin();
	return 0;
}

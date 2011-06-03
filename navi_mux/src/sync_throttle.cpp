#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>

namespace it = image_transport;
namespace mf = message_filters;
using sensor_msgs::Image;

it::Publisher pub_l, pub_r;
int x, y;
int i = 0;

void callback(Image::ConstPtr const &img_l, Image::ConstPtr const &img_r)
{
	// FIXME: don't clump messages when x > 1
	if (i % y >= x) {
		pub_l.publish(img_l);
		pub_r.publish(img_r);
		i = (i + 1) % y;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sync_throttle");
	ros::NodeHandle nh, nh_priv("~");
	std::string const topic_l = nh.resolveName("left");
	std::string const topic_r = nh.resolveName("right");

	// Drop x out of every y messages.
	nh_priv.param<int>("x", x, 1);
	nh_priv.param<int>("y", y, 3);
	if (x > y) {
		ROS_ERROR("ratio of x to y may not exceed 1.0");
		return 1;
	}

	// Throttled outputs.
	it::ImageTransport it(nh);
	pub_l = it.advertise(topic_l + "/image", 1);
	pub_r = it.advertise(topic_r + "/image", 1);

	// Unthrottled inputs.
	// TODO: switch to image_transport SubscriberFilter
	mf::Subscriber<Image> sub_l(nh, topic_l + "/image_fast", 1);
	mf::Subscriber<Image> sub_r(nh, topic_r + "/image_fast", 1);
	mf::TimeSynchronizer<Image, Image> sub_sync(sub_l, sub_r, 10);
	sub_sync.registerCallback(&callback);

	ros::spin();
	return 0;
}

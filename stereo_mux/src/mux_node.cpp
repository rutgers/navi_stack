#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

using image_transport::CameraPublisher;
using image_transport::ImageTransport;
using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using sensor_msgs::Image;
using sensor_msgs::ImageConstPtr;
using message_filters::TimeSynchronizer;

static double const def_fps = 10;

static CameraInfoManager *man_nl, *man_nr;
static CameraInfoManager *man_wl, *man_wr;

static CameraPublisher pub_nl, pub_nr;
static CameraPublisher pub_wl, pub_wr;
static ros::Duration   step;
static ros::Time       latest(0, 0);

void recieve(Image::ConstPtr const &msg_left, Image::ConstPtr const &msg_middle,
             Image::ConstPtr const &msg_right)
{
	// Limit the publishing rate to a maximum sample rate.
	ros::Time now = msg_left->header.stamp;
	if (now - latest < step) return;

	CameraInfo info_nl = man_nl->getCameraInfo();
	CameraInfo info_nr = man_nr->getCameraInfo();
	CameraInfo info_wl = man_wl->getCameraInfo();
	CameraInfo info_wr = man_wr->getCameraInfo();

	// Synchronize the CameraInfo's with their respective images.
	info_nl.header.stamp = now;
	info_nr.header.stamp = now;
	info_wl.header.stamp = now;
	info_wr.header.stamp = now;
	info_nl.header.frame_id = msg_left->.header.frame_id;
	info_nr.header.frame_id = msg_middle->header.frame_id;
	info_wl.header.frame_id = msg_left->header.frame_id;
	info_wr.header.frame_id = msg_right->header.frame_id;

	// Duplicate the left camera with two sets of calibration parameters.
	pub_nl.publish(*msg_left,   info_nl);
	pub_nr.publish(*msg_middle, info_nr);
	pub_wl.publish(*msg_left,   info_wl);
	pub_wr.publish(*msg_right,  info_wr);

	latest = now;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "switcher_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::NodeHandle nh_nl("narrow/left"), nh_nr("narrow/right");
	ros::NodeHandle nh_wl("wide/left"),   nh_wr("wide/right");
	ImageTransport it(nh);

	// Use completely separate calibration parameters for the narrow and wide
	// camera pairs even though one of the cameras is shared. This is
	// necessary to preserve the integrity of the recitification.
	std::string info_nl, info_nr, info_wl, info_wr;
	nh_priv.getParam("info_nl", info_nl);
	nh_priv.getParam("info_nr", info_nr);
	nh_priv.getParam("info_wl", info_wl);
	nh_priv.getParam("info_wr", info_wr);

	man_nl = new CameraInfoManager(nh_nl, "", info_nl);
	man_nr = new CameraInfoManager(nh_nr, "", info_nr);
	man_wl = new CameraInfoManager(nh_wl, "", info_wl);
	man_wr = new CameraInfoManager(nh_wr, "", info_wr);

	// Cap the output FPS at a fixed rate.
	double fps;
	nh_priv.param("fps", fps, def_fps);
	step.fromSec(1 / fps);

	// Publish two pairs of "pseudo-cameras" that multiplex the two available
	// baselines.
	std::string topic_narrow = nh.resolveName("narrow");
	std::string topic_wide   = nh.resolveName("wide");
	pub_nl = it.advertiseCamera(topic_narrow + "/left/image_raw",  10);
	pub_nr = it.advertiseCamera(topic_narrow + "/right/image_raw", 10);
	pub_wl = it.advertiseCamera(topic_wide   + "/left/image_raw",  10);
	pub_wr = it.advertiseCamera(topic_wide   + "/right/image_raw", 10);

	// Always subscribe to synchronized (left, middle, right) image triplets;
	// we will handle the multiplexing entirely in the callback.
	message_filters::Subscriber<Image> sub_left(nh,   "left/image_raw",   1);
	message_filters::Subscriber<Image> sub_middle(nh, "middle/image_raw", 1);
	message_filters::Subscriber<Image> sub_right(nh,  "right/image_raw",  1);
	TimeSynchronizer<Image, Image, Image> sub_sync(sub_left, sub_middle, sub_right, 10);
	sub_sync.registerCallback(boost::bind(&recieve, _1, _2, _3));

	ros::spin();

	delete man_nl;
	delete man_nr;
	delete man_wl;
	delete man_wr;
}

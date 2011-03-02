#include <ros/ros.h>

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

static CameraPublisher pub0;
static CameraPublisher pub1;
static ros::Time       latest(0, 0); 
static ros::Duration   step;
static bool            narrow = true;

void recieve(ImageConstPtr const &msg_img0, CameraInfoConstPtr const &msg_info0,
             ImageConstPtr const &msg_img1, CameraInfoConstPtr const &msg_info1,
             ImageConstPtr const &msg_img2, CameraInfoConstPtr const &msg_info2)
{
	// Limit the publishing rate to a maximum sample rate.
	ros::Time now = msg_img0->header.stamp;
	if (now - latest < step) return;

	if (narrow) {
		pub0.publish(msg_img0, msg_info0);
		pub1.publish(msg_img1, msg_info1);
	} else {
		pub0.publish(msg_img0, msg_info0);
		pub1.publish(msg_img2, msg_info2);
	}

	latest = now;
	narrow = !narrow;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "switcher_node");

	ros::NodeHandle nh, nh_priv("~");
	ImageTransport it(nh);

	// Cap the output FPS at a fixed rate.
	double fps;
	nh_priv.param("fps", fps, def_fps);
	step.fromSec(1 / fps);

	// Publish "pseudo-cameras" that multiplex the available baselines.
	pub0 = it.advertiseCamera("pseudo0/image", 10);
	pub1 = it.advertiseCamera("pseudo1/image", 10);

	// Subscribe to all synchronized images; discard one in the callback.
	message_filters::Subscriber<Image> sub_img0(nh, "camera0/image_raw", 1);
	message_filters::Subscriber<Image> sub_img1(nh, "camera1/image_raw", 1);
	message_filters::Subscriber<Image> sub_img2(nh, "camera2/image_raw", 1);
	message_filters::Subscriber<CameraInfo> sub_info0(nh, "camera0/camera_info", 1);
	message_filters::Subscriber<CameraInfo> sub_info1(nh, "camera1/camera_info", 1);
	message_filters::Subscriber<CameraInfo> sub_info2(nh, "camera2/camera_info", 1);
	TimeSynchronizer<Image, CameraInfo,
	                 Image, CameraInfo,
	                 Image, CameraInfo> sub_sync(sub_img0, sub_info0,
	                                             sub_img1, sub_info1,
	                                             sub_img2, sub_info2, 10);
	sub_sync.registerCallback(boost::bind(&recieve, _1, _2, _3, _4, _5, _6));

	ros::spin();
}

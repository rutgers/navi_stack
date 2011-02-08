#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>

using message_filters::TimeSynchronizer;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

void callback(ImageConstPtr const& img_left, ImageConstPtr const& img_right,
              CameraInfoConstPtr const& info_left, CameraInfoConstPtr const& info_right)
{
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo_image_proc_gpu");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	message_filters::Subscriber<Image> sub_cam_left(nh, "left/image", 1);
	message_filters::Subscriber<Image> sub_cam_right(nh, "left/image", 1);
	message_filters::Subscriber<CameraInfo> sub_info_left(nh, "left/camera_info", 1);
	message_filters::Subscriber<CameraInfo> sub_info_right(nh, "right/camera_info", 1);
	TimeSynchronizer<Image, Image, CameraInfo, CameraInfo> sub_sync(sub_cam_left, sub_cam_right,
	                                                                sub_info_left, sub_info_right);
	sub_sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

	ros::spin();
	return 0;
}

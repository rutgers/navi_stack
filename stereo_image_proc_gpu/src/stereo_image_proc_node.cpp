#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv/opencv.hpp>
#include <opencv/gpu/gpu.hpp>

using cv::gpu::StereoBeliefPropagation;
using message_filters::TimeSynchronizer;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

static cv::gpu::StereoBeliefPropagation stereo;
static cv::gpu::GpuMat img_left;
static cv::gpu::GpuMat img_right;
static cv::gpu::GpuMat img_disparity;
static int prev_width  = -1;
static int prev_height = -1;

void callback(ImageConstPtr const& img_left, ImageConstPtr const& img_right,
              CameraInfoConstPtr const& info_left, CameraInfoConstPtr const& info_right)
{
	if (img_left.width != info_left.width || img_right.width != img_right.height) {
		ROS_ERROR("calibration parameters have incorrect resolution");
		return;
	} else if (img_left.width != img_right.width || img_left.height != img_right.height) {
		ROS_ERROR("left and right images have different resolution");
		return;
	}

	// Recompute the parameters for the matching algorithm whenever the image
	// resolution changes. Also resizes the necessary image buffers.
	int const width  = img_left.width;
	int const height = img_left.height;

	if (prev_width != width || prev_height != height) {
		int ndisp, iters, levels;
		StereoBeliefPropagation::estimateRecommendedParams(width, height, ndisp, iters, levels);
		stereo = StereoBeliefPropagation(ndisp, iters, levels, CV_8UC3);

		img_left.create(height, width, CV_8UC3);
		img_right.create(height, width, CV_8UC3);

		prev_width  = width;
		prev_height = height;
	}

	// TODO: Convert the Image messages into GpuMat objects.
	// TODO: Rectify images as preparation for using StereoBeliefPropagation.

	stereo(img_left, img_right, img_disparity);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo_image_proc_gpu");
	ros::NodeHandle nh;

	// TODO: Use estimateRecommendedParams() to estimate the parameters for the
	//       StereoBeliefPropagation class.
#if 0
	int ndisp, iters, levels;
	StereoBeliefPropagation::estimateRecommendedParams(width, height, ndisp, iters, levels);
#endif

	// Synchronize the two webcams' images and calibration parameters.
	// TODO: Attempt to synchronize two image_transport::CameraSubscribers.
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

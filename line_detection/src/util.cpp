#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "util.hpp"

using geometry_msgs::PointStamped;
using geometry_msgs::Vector3Stamped;

void LineColorTransform(cv::Mat src, cv::Mat &dst, bool invert)
{
	ROS_ASSERT(src.type() == CV_8UC3);

	// Blur the original image to reduce noise in the grass.
	// TODO: Make the radius a parameter.
	cv::Mat src_blur;
	//cv::GaussianBlur(src, src_blur, cv::Size(3, 3), 0.0);
	src_blur = src;

	// Convert to the HSV color space to get saturation and intensity.
	std::vector<cv::Mat> img_chan;
	cv::Mat img_hsv;
	cv::cvtColor(src_blur, img_hsv, CV_BGR2HSV);
	cv::split(img_hsv, img_chan);

	// Find bright (i.e. high intensity) regions of little color (i.e. low
	// saturation) using the minimum operator.
	// TODO: Find a better way of doing this transformation.
	cv::Mat img_sat = 255 - img_chan[1];
	cv::Mat img_val = img_chan[2];

	// Detect black lines on a bright surface for testing.
	if (invert) {
		img_val = 255 - img_val;
	}

	cv::Mat dst_8u;
	cv::min(img_sat, img_val, dst_8u);
	dst_8u.convertTo(dst, CV_64FC1);
}

void GuessGroundPlane(tf::TransformListener &tf,
                      std::string fr_gnd, std::string fr_cam,
                      Plane &plane)
{
	// Assume the origin of the base_footprint frame is on the ground plane.
	PointStamped point_gnd, point_cam;
	point_gnd.header.stamp    = ros::Time(0);
	point_gnd.header.frame_id = fr_gnd;
	point_gnd.point.x = 0.0;
	point_gnd.point.y = 0.0;
	point_gnd.point.z = 0.0;

	// Assume the positive z-axis of the base_footprint frame is "up", implying
	// that it is normal to the ground plane.
	Vector3Stamped normal_gnd, normal_cam;
	normal_gnd.header.stamp    = ros::Time(0);
	normal_gnd.header.frame_id = fr_gnd;
	normal_gnd.vector.x = 0.0;
	normal_gnd.vector.y = 0.0;
	normal_gnd.vector.z = 1.0;

	// Forward direction of the robot.
	Vector3Stamped forward_gnd, forward_cam;
	forward_gnd.header.stamp    = ros::Time(0);
	forward_gnd.header.frame_id = fr_gnd;
	forward_gnd.vector.x = 1.0;
	forward_gnd.vector.y = 0.0;
	forward_gnd.vector.z = 0.0;

	// These may throw a tf::TransformException.
	tf.transformPoint(fr_cam, point_gnd, point_cam);
	tf.transformVector(fr_cam, normal_gnd, normal_cam);
	tf.transformVector(fr_cam, forward_gnd, forward_cam);

	// Convert from a StampedVector to the OpenCV data type.
	plane.point.x   = point_cam.point.x;
	plane.point.y   = point_cam.point.y;
	plane.point.z   = point_cam.point.z;
	plane.normal.x  = normal_cam.vector.x;
	plane.normal.y  = normal_cam.vector.y;
	plane.normal.z  = normal_cam.vector.z;
	plane.forward.x = forward_cam.vector.x;
	plane.forward.y = forward_cam.vector.y;
	plane.forward.z = forward_cam.vector.z;
}

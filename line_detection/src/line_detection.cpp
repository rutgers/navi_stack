#include <cmath>
#include <string>
#include <vector>

#include <opencv/cv.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

#include "util.hpp"

#define DEBUG

namespace image_encodings = sensor_msgs::image_encodings;

using geometry_msgs::PointStamped;
using geometry_msgs::Vector3Stamped;
using image_transport::CameraSubscriber;
using sensor_msgs::CameraInfoConstPtr;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

static CameraSubscriber sub_cam;
static ros::Publisher   pub_pts;
static tf::TransformListener *listener;

#ifdef DEBUG
static image_transport::Publisher pub_debug_filt;
static image_transport::Publisher pub_debug_hor;
static image_transport::Publisher pub_debug_ver;
static image_transport::Publisher pub_debug_combo;
static image_transport::Publisher pub_debug_max;
static image_transport::Publisher pub_debug_overlay;
#endif

static std::string p_frame;
static double p_thickness;
static double p_threshold;
static double p_border;

void GuessGroundPlane(std::string fr_gnd, std::string fr_cam, Plane &plane)
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

	// These may throw a tf::TransformException.
	listener->transformPoint(fr_cam, point_gnd, point_cam);
	listener->transformVector(fr_cam, normal_gnd, normal_cam);

	// Convert from a StampedVector to the OpenCV data type.
	plane.point.x  = point_cam.point.x;
	plane.point.y  = point_cam.point.y;
	plane.point.z  = point_cam.point.z;
	plane.normal.x = normal_cam.vector.x;
	plane.normal.y = normal_cam.vector.y;
	plane.normal.z = normal_cam.vector.z;
}

void callback(ImageConstPtr const &msg_img, CameraInfoConstPtr const &msg_cam)
{
	// Estimate the ground plane using the /base_footprint tf frame.
	Plane plane;
	std::string ground_id = p_frame;
	std::string camera_id = msg_img->header.frame_id;

	try {
		GuessGroundPlane(ground_id, camera_id, plane);
	} catch (tf::TransformException ex) {
		ROS_ERROR_THROTTLE(30, "%s", ex.what());
		return;
	}

	// Convert ROS message formats to OpenCV data types.
	cv_bridge::CvImagePtr img_ptr;
	try {
		img_ptr = cv_bridge::toCvCopy(msg_img, image_encodings::BGR8);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR_THROTTLE(30, "%s", e.what());
		return;
	}
	cv::Mat &img_input = img_ptr->image;
	cv::Mat mint;
	CameraInfoToMat(msg_cam, mint);

	// Find local maxima in the response of a matched pulse-width filter.
	std::list<cv::Point2i> maxima;
	cv::Mat img_white, img_hor, img_ver;
	LineColorTransform(img_input, img_white);
	LineFilter(img_white, img_hor, img_ver, mint, plane, p_thickness, p_border);
	FindMaxima(img_hor, img_ver, maxima, p_threshold);

	// Publish a three-dimensional point cloud in the camera frame by converting
	// each maximum's pixel coordinates to camera coordinates using the camera's
	// intrinsics and knowledge of the ground plane.
	sensor_msgs::PointCloud pts_msg;
	std::list<cv::Point2i>::const_iterator it;

	for (it = maxima.begin(); it != maxima.end(); ++it) {
		cv::Point2i pt_image(*it);
		cv::Point3d ray, pt_world;

		GetPixelRay(mint, pt_image, ray);
		GetRayPlaneInt(ray, plane, pt_world);

		// Convert OpenCV cv::Point into a ROS geometry_msgs::Point object.
		geometry_msgs::Point32 pt_msg;
		pt_msg.x = (float)pt_world.x;
		pt_msg.y = (float)pt_world.y;
		pt_msg.z = (float)pt_world.z;
		pts_msg.points.push_back(pt_msg);
	}

	pts_msg.header.stamp    = msg_img->header.stamp;
	pts_msg.header.frame_id = msg_img->header.frame_id;
	pub_pts.publish(pts_msg);

#ifdef DEBUG
#if 0
	// Color space transformation.
	cv_bridge::CvImage msg_blur;
	msg_blur.header.stamp    = msg_img->header.stamp;
	msg_blur.header.frame_id = msg_img->header.frame_id;
	msg_blur.encoding = image_encodings::MONO8;
	msg_blur.image    = img_white;
	pub_blur.publish(msg_blur.toImageMsg());
#endif

	// Calculate the expected width of a line in each row of the image. Stop
	// at the horizon line by detecting an increase in pixel width.
	double width_old = INFINITY;

	cv::Mat img_debug(img_input.rows, img_input.cols, CV_64FC1, cv::Scalar(0)); 

	for (int y = img_debug.rows - 1; y >= 0; --y) {
		cv::Point2d mid(img_debug.cols / 2.0, y);
		double width  = GetDistSize(mid, p_thickness, mint, plane);
		double border = GetDistSize(mid, p_border,    mint, plane);

		if (width >= width_old) break;
		width_old = width;

		cv::Mat row = img_debug.row(y);
		BuildLineFilter(mid.x, img_debug.cols, width, border, row);
	}

	cv::Mat img_debug_8u;
	cv::normalize(img_debug, img_debug_8u, 0, 255, CV_MINMAX, CV_8UC1);

	cv_bridge::CvImage msg_debug_filt;
	msg_debug_filt.header.stamp    = msg_img->header.stamp;
	msg_debug_filt.header.frame_id = msg_img->header.frame_id;
	msg_debug_filt.encoding = image_encodings::MONO8;
	msg_debug_filt.image    = img_debug_8u;
	pub_debug_filt.publish(msg_debug_filt.toImageMsg());

	// Response of the horizontal matched pulse-width filter.
	cv::Mat img_debug_hor;
	cv::normalize(img_hor, img_debug_hor, 0, 255, CV_MINMAX, CV_8UC1);

	cv_bridge::CvImage msg_debug_hor;
	msg_debug_hor.header.stamp    = msg_img->header.stamp;
	msg_debug_hor.header.frame_id = msg_img->header.frame_id;
	msg_debug_hor.encoding = image_encodings::MONO8;
	msg_debug_hor.image    = img_debug_hor;
	pub_debug_hor.publish(msg_debug_hor.toImageMsg());

	// Response of the vertical matched pulse-width filter.
	cv::Mat img_debug_ver;
	cv::normalize(img_ver, img_debug_ver, 0, 255, CV_MINMAX, CV_8UC1);

	cv_bridge::CvImage msg_debug_ver;
	msg_debug_ver.header.stamp    = msg_img->header.stamp;
	msg_debug_ver.header.frame_id = msg_img->header.frame_id;
	msg_debug_ver.encoding = image_encodings::MONO8;
	msg_debug_ver.image    = img_debug_ver;
	pub_debug_ver.publish(msg_debug_ver.toImageMsg());

	// Combination of the two filter responses.
	cv::Mat img_debug_combo_tmp = img_debug_hor + img_debug_ver;
	cv::Mat img_debug_combo;
	cv::normalize(img_debug_combo_tmp, img_debug_combo, 0, 255, CV_MINMAX, CV_8UC1);

	cv_bridge::CvImage msg_debug_combo;
	msg_debug_combo.header.stamp    = msg_img->header.stamp;
	msg_debug_combo.header.frame_id = msg_img->header.frame_id;
	msg_debug_combo.encoding = image_encodings::MONO8;
	msg_debug_combo.image    = img_debug_combo;
	pub_debug_combo.publish(msg_debug_combo.toImageMsg());

	// Non-maximal supression.
	cv::Mat img_maxima(img_input.rows, img_input.cols, CV_8U, cv::Scalar(0));
	for (it = maxima.begin(); it != maxima.end(); ++it) {
		img_maxima.at<uint8_t>(it->y, it->x) = 255;
	}

	cv_bridge::CvImage msg_debug_max;
	msg_debug_max.header.stamp    = msg_img->header.stamp;
	msg_debug_max.header.frame_id = msg_img->header.frame_id;
	msg_debug_max.encoding = image_encodings::MONO8;
	msg_debug_max.image    = img_maxima;
	pub_debug_max.publish(msg_debug_max.toImageMsg());

	// Overlay maxima on the original image.
	cv::Mat img_overlay = img_input.clone();
	std::vector<cv::Mat> img_overlay_chan;
	cv::split(img_overlay, img_overlay_chan);

	for (it = maxima.begin(); it != maxima.end(); ++it) {
		img_overlay_chan[0].at<uint8_t>(it->y, it->x) = 0;
		img_overlay_chan[1].at<uint8_t>(it->y, it->x) = 0;
		img_overlay_chan[2].at<uint8_t>(it->y, it->x) = 255;
	}

	cv::merge(img_overlay_chan, img_overlay);

	cv_bridge::CvImage msg_debug_overlay;
	msg_debug_overlay.header.stamp    = msg_img->header.stamp;
	msg_debug_overlay.header.frame_id = msg_img->header.frame_id;
	msg_debug_overlay.encoding = image_encodings::BGR8;
	msg_debug_overlay.image    = img_overlay;
	pub_debug_overlay.publish(msg_debug_overlay.toImageMsg());
#endif
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_detection");
	ros::NodeHandle nh;

	listener = new tf::TransformListener;

	nh.param<double>("thickness", p_thickness, 0.0726);
	nh.param<double>("border",    p_border,    0.1452);
	nh.param<double>("threshold", p_threshold, 40);
	nh.param<std::string>("frame", p_frame, "base_footprint");

	image_transport::ImageTransport it(nh);
	sub_cam = it.subscribeCamera("image", 1, &callback);
	pub_pts = nh.advertise<sensor_msgs::PointCloud>("line_points", 10);

#ifdef DEBUG
	pub_debug_filt    = it.advertise("debug_filt",    10);
	pub_debug_hor     = it.advertise("debug_hor",     10);
	pub_debug_ver     = it.advertise("debug_ver",     10);
	pub_debug_combo   = it.advertise("debug_combo",   10);
	pub_debug_max     = it.advertise("debug_max",     10);
	pub_debug_overlay = it.advertise("debug_overlay", 10);
#endif

	ros::spin();
	return 0;
}

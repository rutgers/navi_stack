#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "util.hpp"

using geometry_msgs::PointStamped;
using geometry_msgs::Vector3Stamped;

void GetPixelRay(cv::Mat mint, cv::Point2i pt, cv::Point3d &dst)
{
	ROS_ASSERT(mint.rows == 3 && mint.cols == 3);
	ROS_ASSERT(mint.type() == CV_64FC1);

	// Convert the image coordinates into a homogenous coordinate system.
	cv::Mat p_img(3, 1, CV_64FC1);
	p_img.at<double>(0, 0) = pt.x;
	p_img.at<double>(1, 0) = pt.y;
	p_img.at<double>(2, 0) = 1.0;

	// Transform into the camera coordinate frame using the intrinsics.
	cv::Mat p_cam = mint.inv() * p_img;
	dst.x = p_cam.at<double>(0, 0);
	dst.y = p_cam.at<double>(1, 0);
	dst.z = p_cam.at<double>(2, 0);
}

void GetRayPlaneInt(cv::Point3d ray, Plane plane, cv::Point3d &dst)
{
	dst = (plane.normal.dot(plane.point) / plane.normal.dot(ray)) * ray;
}

double GetDistSize(cv::Point2d pt0, cv::Point3d dist, cv::Mat mint, Plane plane)
{
	ROS_ASSERT(mint.rows == 3 && mint.cols == 3);
	ROS_ASSERT(mint.type() == CV_64FC1);

	// Find the world coordiante of the point on the plane corresponding to
	// the given pixel coordinates in the image.
	cv::Point3d ray, pt0_world;
	GetPixelRay(mint, pt0, ray);
	GetRayPlaneInt(ray, plane, pt0_world);

	// Calculate the image coordinates of a world point offset by the
	// appropriate number of pixels in the plane (using the cross product).
	cv::Mat pt1_world(3, 1, CV_64FC1);
	pt1_world.at<double>(0, 0) = pt0_world.x - 0.5 * dist.x;
	pt1_world.at<double>(1, 0) = pt0_world.y - 0.5 * dist.y;
	pt1_world.at<double>(2, 0) = pt0_world.z - 0.5 * dist.z;
	cv::Mat pt1_mat = mint * pt1_world;

	cv::Mat pt2_world(3, 1, CV_64FC1);
	pt2_world.at<double>(0, 0) = pt0_world.x + 0.5 * dist.x;
	pt2_world.at<double>(1, 0) = pt0_world.y + 0.5 * dist.y;
	pt2_world.at<double>(2, 0) = pt0_world.z + 0.5 * dist.z;
	cv::Mat pt2_mat = mint * pt2_world;

	// Project this offset point into the image.
	cv::Point2d pt1(pt1_mat.at<double>(0, 0), pt1_mat.at<double>(1, 0));
	pt1 *= 1.0 / pt1_mat.at<double>(2, 0);

	cv::Point2d pt2(pt2_mat.at<double>(0, 0), pt2_mat.at<double>(1, 0));
	pt2 *= 1.0 / pt2_mat.at<double>(2, 0);

	// Find the distance from the offset point to the original point.
	cv::Point2d offset = pt2 - pt1;
	return sqrt(offset.dot(offset));
}

double GetPixSize(cv::Point2d pt0, cv::Point2d dist, cv::Mat mint, Plane plane)
{
	cv::Point3d ray0, ray1;
	GetPixelRay(mint, pt0,        ray0);
	GetPixelRay(mint, pt0 + dist, ray1);

	cv::Point3d pt0_world, pt1_world;
	GetRayPlaneInt(ray0, plane, pt0_world);
	GetRayPlaneInt(ray1, plane, pt1_world);

	cv::Point3d offset = pt1_world - pt0_world;
	return sqrt(offset.dot(offset));
}

void CameraInfoToMat(CameraInfoConstPtr const &msg, cv::Mat &mint)
{
	mint.create(3, 3, CV_64FC1);

	for (int i = 0; i < 3; ++i)
	for (int j = 0; j < 3; ++j) {
		mint.at<double>(i, j) = msg->K[i * 3 + j];
	}
}

void BuildLineFilter(cv::Mat &ker, int x, int dim, int width, int border, int &fwidth, bool fit)
{
	ROS_ASSERT(0 <= x && x < dim);
	ROS_ASSERT(dim > 0);

	cv::Range range_rows(0, 1);
	cv::Range range_lo_l;
	cv::Range range_lo_r;
	cv::Range range_hi;

	fwidth = border + width + border;

	// Only allocate as much space is necessary.
	if (fit) {
		ROS_ASSERT(ker.rows == 1 && ker.cols >= width + 2 * border);
		ROS_ASSERT(ker.type() == CV_64FC1);

		range_lo_l = cv::Range(0, border);
		range_hi   = cv::Range(border, border + width);
		range_lo_r = cv::Range(border + width, border + width + border);
	}
	// Allocate a full row to make the dot multiplication simpler.
	else {
		ker.create(1, dim, CV_64FC1);
		ker.setTo(0.0);

		int x1 = MAX(x - (width + 0) / 2 - border, 0);       // falling edge, trough
		int x2 = MAX(x - (width + 0) / 2,          0);       // rising edge,  peak
		int x3 = MIN(x + (width + 1) / 2,          dim - 1); // falling edge, peak
		int x4 = MIN(x + (width + 1) / 2 + border, dim - 1); // rising edge,  trough

		range_lo_l = cv::Range(x1, x2);
		range_hi   = cv::Range(x2, x3);
		range_lo_r = cv::Range(x3, x4);
	}

	cv::Mat lo_l = ker(range_rows, range_lo_l);
	cv::Mat hi   = ker(range_rows, range_hi);
	cv::Mat lo_r = ker(range_rows, range_lo_r);

	lo_l.setTo(-0.5 / lo_l.cols);
	hi.setTo(  +1.0 / hi.cols);
	lo_r.setTo(-0.5 / lo_r.cols);
}

void LineColorTransform(cv::Mat src, cv::Mat &dst)
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

	cv::Mat dst_8u;
	cv::min(img_sat, img_val, dst_8u);
	dst_8u.convertTo(dst, CV_64FC1);
}

void NormalizeSaturation(cv::Mat src, cv::Mat &dst, cv::Size size)
{
	ROS_ASSERT(src.type() == CV_8UC3);
	ROS_ASSERT(size.width > 0 && size.width % 2 == 1);
	ROS_ASSERT(size.height > 0 && size.height % 2 == 1);

	int off_hor = size.width / 2;
	int off_ver = size.height / 2;

	// Convert the image into the HSV color space to get direct access to its
	// per-pixel saturation.
	std::vector<cv::Mat> img_chan;
	cv::Mat img_hsv;
	cv::cvtColor(src, img_hsv, CV_BGR2HSV);
	cv::split(img_hsv, img_chan);

	cv::Mat sat_new(src.rows, src.cols, CV_8U, cv::Scalar(0));
	cv::Mat sat_old = img_chan[1];

	for (int r0 = off_ver; r0 < src.rows - off_ver; ++r0)
	for (int c0 = off_hor; c0 < src.cols - off_hor; ++c0) {
		uint8_t  val_old = sat_old.at<uint8_t>(r0, c0);
		uint8_t &val_new = sat_new.at<uint8_t>(r0, c0);

		// Find the minimum and maximum saturation in a window.
		cv::Range range_ver(r0 - off_ver, r0 + off_ver);
		cv::Range range_hor(c0 - off_hor, c0 + off_hor);
		cv::Mat window = sat_new(range_ver, range_hor);

		double sat_min, sat_max;
		cv::minMaxLoc(window, &sat_min, &sat_max);

		val_new = (uint8_t)((val_old - sat_min) * 255.0 / sat_max);
	}

	std::vector<cv::Mat> img_chan_out(3);
	img_chan_out[0] = img_chan[0];
	img_chan_out[1] = sat_new;
	img_chan_out[2] = img_chan[2];

	cv::Mat dst_hsv;
	cv::merge(img_chan_out, dst_hsv);
	cv::cvtColor(dst_hsv, dst, CV_HSV2BGR);
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

	// These may throw a tf::TransformException.
	tf.transformPoint(fr_cam, point_gnd, point_cam);
	tf.transformVector(fr_cam, normal_gnd, normal_cam);

	// Convert from a StampedVector to the OpenCV data type.
	plane.point.x  = point_cam.point.x;
	plane.point.y  = point_cam.point.y;
	plane.point.z  = point_cam.point.z;
	plane.normal.x = normal_cam.vector.x;
	plane.normal.y = normal_cam.vector.y;
	plane.normal.z = normal_cam.vector.z;
}

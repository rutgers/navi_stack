#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include "LineDetectionNode.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

using visualization_msgs::Marker;

// TODO: Convert the direction of principal curvature to real-world coordinates.

LineDetectionNode::LineDetectionNode(ros::NodeHandle nh, std::string ground_id,
                                     bool debug)
	: m_debug(debug),
	  m_invert(false),
	  m_valid(false),
	  m_num_prev(0),
	  m_ground_id(ground_id),
	  m_nh(nh)
{
	image_transport::ImageTransport it(nh);

	m_sub_cam = it.subscribeCamera("image", 1, &LineDetectionNode::ImageCallback, this);
	m_pub_pts = m_nh.advertise<PointCloudXYZ>("line_points", 10);

	if (m_debug) {
		ROS_WARN("debugging topics are enabled; performance may be degraded");

		m_pub_pre        = it.advertise("line_pre",        10);
		m_pub_distance   = it.advertise("line_distance",   10);
		m_pub_ker_hor    = it.advertise("line_kernel_hor", 10);
		m_pub_ker_ver    = it.advertise("line_kernel_ver", 10);
		m_pub_filter_hor = it.advertise("line_filter_hor", 10);
		m_pub_filter_ver = it.advertise("line_filter_ver", 10);
		m_pub_visual_one = m_nh.advertise<Marker>("/visualization_marker", 1);
	}
}

void LineDetectionNode::SetCutoffWidth(int width_cutoff)
{
	ROS_ASSERT(width_cutoff > 0);

	m_valid        = m_valid && (width_cutoff == m_width_cutoff);
	m_width_cutoff = width_cutoff;
}

void LineDetectionNode::SetDeadWidth(double width_dead)
{
	ROS_ASSERT(width_dead > 0.0);

	m_valid      = m_valid && (width_dead == m_width_dead);
	m_width_dead = width_dead;
}

void LineDetectionNode::SetLineWidth(double width_line)
{
	ROS_ASSERT(width_line > 0.0);

	m_valid      = m_valid && (width_line == m_width_line);
	m_width_line = width_line;
}

void LineDetectionNode::SetInvert(bool invert) {
	m_invert = invert;
}

void LineDetectionNode::SetGroundPlane(Plane plane)
{
	m_valid = m_valid && (plane.point.x == m_plane.point.x)
	                  && (plane.point.y == m_plane.point.y)
	                  && (plane.point.z == m_plane.point.z)
	                  && (plane.normal.x == m_plane.normal.x)
	                  && (plane.normal.y == m_plane.normal.y)
	                  && (plane.normal.z == m_plane.normal.z)
	                  && (plane.forward.x == m_plane.forward.x)
	                  && (plane.forward.y == m_plane.forward.y)
	                  && (plane.forward.z == m_plane.forward.z);
	m_plane = plane;
}

void LineDetectionNode::SetThreshold(double threshold)
{
	m_valid     = m_valid && (threshold == m_threshold);
	m_threshold = threshold;
}

void LineDetectionNode::SetResolution(int width, int height)
{
	ROS_ASSERT(width > 0 && height > 0);

	m_valid = m_valid && (width == m_cols) && (height == m_rows);
	m_cols  = width;
	m_rows  = height;
}

void LineDetectionNode::NonMaxSupr(cv::Mat src_hor, cv::Mat src_ver, PointCloudXYZ &dst)
{
	ROS_ASSERT(src_hor.rows == src_ver.rows && src_hor.cols == src_ver.cols);
	ROS_ASSERT(src_hor.type() == CV_64FC1 && src_ver.type() == CV_64FC1);
	ROS_ASSERT(m_valid);

	float nan = std::numeric_limits<float>::quiet_NaN();

	dst.width    = src_hor.cols;
	dst.height   = src_hor.rows;
	dst.is_dense = false;
	dst.points.resize(src_hor.cols * src_hor.rows);

	for (int y = 1; y < src_hor.rows - 1; ++y)
	for (int x = 1; x < src_hor.cols - 1; ++x) {
		pcl::PointXYZ &pt = dst.points[y * src_hor.cols + x];

		double val_hor   = src_hor.at<double>(y, x);
		double val_left  = src_hor.at<double>(y, x - 1);
		double val_right = src_hor.at<double>(y, x + 1);

		double val_ver = src_ver.at<double>(y + 0, x);
		double val_top = src_ver.at<double>(y - 1, x);
		double val_bot = src_ver.at<double>(y + 1, x);

		bool is_hor = val_hor > val_left && val_hor > val_right && val_hor > m_threshold;
		bool is_ver = val_ver > val_top  && val_ver > val_bot   && val_ver > m_threshold;

		// Found a line; project it into 3D using the ground plane.
		if (is_hor || is_ver) {
			cv::Point3d pt_3d = GetGroundPoint(cv::Point2d(x, y));
			pt.x = pt_3d.x;
			pt.y = pt_3d.y;
			pt.z = pt_3d.z;
		}
		// Fill areas that are "not line" with NaN.
		else {
			pt.x = nan;
			pt.y = nan;
			pt.z = nan;
		}
	}
}

void LineDetectionNode::UpdateCache(void)
{
	static cv::Point3d const dhor(1.0, 0.0, 0.0);
	static cv::Point3d const dver(0.0, 0.0, 1.0);

	ROS_ASSERT(m_width_line > 0.0);
	ROS_ASSERT(m_width_dead > 0.0);
	ROS_ASSERT(m_width_cutoff > 0);

	if (!m_valid) {
		ROS_INFO("rebuilding cache with changed parameters");
		ROS_INFO("found ground plane P(%4f, %4f, %4f) N(%4f, %f, %f)",
			m_plane.point.x,  m_plane.point.y,  m_plane.point.z,
			m_plane.normal.x, m_plane.normal.y, m_plane.normal.z
		);

		m_horizon_hor = GeneratePulseFilter(dhor, m_kernel_hor, m_offset_hor);
		m_horizon_ver = GeneratePulseFilter(dhor, m_kernel_ver, m_offset_ver);
		//m_horizon_ver = GeneratePulseFilter(dver, m_kernel_ver, m_offset_ver);

		ROS_INFO("detected horizon horizontal = %d, vertical = %d",
			m_horizon_hor, m_horizon_ver
		);
	}
	m_valid = true;
}

void LineDetectionNode::ImageCallback(ImageConstPtr const &msg_img,
                                      CameraInfoConstPtr const &msg_cam)
{
	namespace enc = sensor_msgs::image_encodings;

	// Keep the ground plane in sync with the latest TF data.
	Plane plane;
	try {
		std::string ground_id = m_ground_id;
		std::string camera_id = msg_img->header.frame_id;
		GuessGroundPlane(m_tf, ground_id, camera_id, msg_img->header.stamp, plane);
	} catch (tf::TransformException ex) {
		ROS_ERROR_THROTTLE(30, "%s", ex.what());
		return;
	}

	// Convert the ROS Image and CameraInfo messages into OpenCV datatypes for
	// processing. This avoids copying the data when possible.
	cv::Mat img_src;
	try {
		m_model.fromCameraInfo(msg_cam);
		cv_bridge::CvImageConstPtr src_tmp = cv_bridge::toCvShare(msg_img, enc::MONO8);

		// TODO: Directly process the 8-bit image to avoid this type conversion.
		cv::Mat img_src8 = src_tmp->image;
		img_src8.convertTo(img_src, CV_64FC1);
	} catch (cv_bridge::Exception &e) {
		ROS_WARN_THROTTLE(10, "unable to parse image message");
		return;
	}

	// Update cached values. If any values change, the filter kernel will be
	// recomputed.
	SetGroundPlane(plane);
	SetResolution(msg_img->width, msg_img->height);
	UpdateCache();

	cv::Mat img_hor, img_ver;
	PulseFilter(img_src, img_hor, m_kernel_hor, m_offset_hor, true);
	PulseFilter(img_src, img_ver, m_kernel_ver, m_offset_ver, false);

	PointCloudXYZ maxima;
	NonMaxSupr(img_hor, img_ver, maxima);

	sensor_msgs::PointCloud2 msg_maxima;
	pcl::toROSMsg(maxima, msg_maxima);
	msg_maxima.header.stamp    = msg_img->header.stamp;
	msg_maxima.header.frame_id = msg_img->header.frame_id;
	m_pub_pts.publish(msg_maxima);

	if (m_debug) {
		// Render lines every 1 m on the ground plane and render them in 3D!
		cv::Mat img_distance  = img_src.clone();
		cv::Point3d P_ground  = m_plane.point;
		cv::Point3d P_forward = m_plane.forward;
		P_forward *= 1.0 / sqrt(P_forward.dot(P_forward));

		double z_step = 1;
		double z_max  = 1000;

		Marker msg_contour;
		msg_contour.header.stamp    = msg_img->header.stamp;
		msg_contour.header.frame_id = msg_img->header.frame_id;
		msg_contour.ns     = "line_contour";
		msg_contour.id     = 0;
		msg_contour.type   = Marker::LINE_LIST;
		msg_contour.action = Marker::ADD;
		msg_contour.points.resize(2 * z_max);
		msg_contour.scale.x = 0.05;
		msg_contour.color.r = 1.0;
		msg_contour.color.g = 0.0;
		msg_contour.color.b = 0.0;
		msg_contour.color.a = 1.0;

		for (int i = 0; i < (int)(z_max / z_step); ++i) {
			P_ground += z_step * P_forward;
			cv::Point2d p = m_model.project3dToPixel(P_ground);
			cv::Point2d p1(0.0, p.y);
			cv::Point2d p2(m_cols, p.y);
			cv::line(img_distance, cv::Point2d(0.0, p.y), cv::Point2d(m_cols, p.y), cv::Scalar(255, 0, 0), 1);

			// 3D Marker
			msg_contour.points[2 * i + 0].x = P_ground.x - 1.0;
			msg_contour.points[2 * i + 0].y = P_ground.y;
			msg_contour.points[2 * i + 0].z = P_ground.z;
			msg_contour.points[2 * i + 1].x = P_ground.x + 1.0;
			msg_contour.points[2 * i + 1].y = P_ground.y;
			msg_contour.points[2 * i + 1].z = P_ground.z;
		}

		cv_bridge::CvImage msg_distance;
		msg_distance.header.stamp    = msg_img->header.stamp;
		msg_distance.header.frame_id = msg_img->header.frame_id;
		msg_distance.encoding = enc::BGR8;
		msg_distance.image    = img_distance;
		m_pub_distance.publish(msg_distance.toImageMsg());
		m_pub_visual_one.publish(msg_contour);

		// Visualize the matched pulse width kernels.
		cv::Mat img_ker_hor;
		cv::normalize(m_kernel_hor, img_ker_hor, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_ker_hor;
		msg_ker_hor.header.stamp    = msg_img->header.stamp;
		msg_ker_hor.header.frame_id = msg_img->header.frame_id;
		msg_ker_hor.encoding = enc::MONO8;
		msg_ker_hor.image    = img_ker_hor;
		m_pub_ker_hor.publish(msg_ker_hor.toImageMsg());

		cv::Mat img_ker_ver;
		cv::normalize(m_kernel_ver, img_ker_ver, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_ker_ver;
		msg_ker_ver.header.stamp    = msg_img->header.stamp;
		msg_ker_ver.header.frame_id = msg_img->header.frame_id;
		msg_ker_ver.encoding = enc::MONO8;
		msg_ker_ver.image    = img_ker_ver;
		m_pub_ker_ver.publish(msg_ker_ver.toImageMsg());

		// Visualize the raw filter responses.
		cv::Mat img_filter_hor;
		cv::normalize(img_hor, img_filter_hor, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_filter_hor;
		msg_filter_hor.header.stamp    = msg_img->header.stamp;
		msg_filter_hor.header.frame_id = msg_img->header.frame_id;
		msg_filter_hor.encoding = enc::MONO8;
		msg_filter_hor.image    = img_filter_hor;
		m_pub_filter_hor.publish(msg_filter_hor.toImageMsg());

		cv::Mat img_filter_ver;
		cv::normalize(img_ver, img_filter_ver, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_filter_ver;
		msg_filter_ver.header.stamp    = msg_img->header.stamp;
		msg_filter_ver.header.frame_id = msg_img->header.frame_id;
		msg_filter_ver.encoding = enc::MONO8;
		msg_filter_ver.image    = img_filter_ver;
		m_pub_filter_ver.publish(msg_filter_ver.toImageMsg());
	}
}

cv::Point3d LineDetectionNode::GetGroundPoint(cv::Point2d pt)
{
	cv::Point3d ray     = m_model.projectPixelTo3dRay(pt);
	cv::Point3d &normal = m_plane.normal;
	cv::Point3d &plane  = m_plane.point;
	return (plane.dot(normal) / ray.dot(normal)) * ray;
}

double LineDetectionNode::ProjectDistance(cv::Point2d pt, cv::Point3d offset)
{
	// Project the expected edge points back into the image.
	cv::Point3d P = GetGroundPoint(pt);
	cv::Point2d p1 = m_model.project3dToPixel(P);
	cv::Point2d p2 = m_model.project3dToPixel(P + offset);

	// Find the distance between the reprojected points.
	cv::Point2d diff = p2 - p1;
	return sqrt(diff.dot(diff));
}

double LineDetectionNode::ReprojectDistance(cv::Point2d pt, cv::Point2d offset)
{
	cv::Point3d P1 = GetGroundPoint(pt);
	cv::Point3d P2 = GetGroundPoint(pt + offset);

	cv::Point3d diff = P2 - P1;
	return sqrt(diff.dot(diff));
}

int LineDetectionNode::GeneratePulseFilter(cv::Point3d dw, cv::Mat &kernel, std::vector<Offset> &offsets)
{
	static Offset const offset_template = { 0, 0 };

	ROS_ASSERT(m_rows > 0 && m_cols > 0);
	ROS_ASSERT(m_width_line > 0.0);
	ROS_ASSERT(m_width_dead > 0.0);

	kernel.create(m_rows, m_cols, CV_64FC1);
	kernel.setTo(0.0);

	offsets.clear();
	offsets.resize(m_rows, offset_template);

	int width_prev = INT_MAX;
	int horizon    = m_rows - 1;

	for (int r = m_rows - 1; r >= 0; --r) {
		cv::Point2d middle(m_cols / 2, r);
		int offs_line_neg = ProjectDistance(middle, -0.5 * m_width_line * dw);
		int offs_line_pos = ProjectDistance(middle, +0.5 * m_width_line * dw);
		int offs_both_neg = ProjectDistance(middle, -0.5 * (m_width_line + 2.0 * m_width_dead) * dw);
		int offs_both_pos = ProjectDistance(middle, +0.5 * (m_width_line + 2.0 * m_width_dead) * dw);

		int width_line = offs_line_neg + offs_line_pos;
		int width_both = offs_both_neg + offs_both_pos;
		int width_dead = width_both - width_line;
		int width_min  = std::min(width_line, width_dead);

		// Only generate a kernel when both the filter's pulse and supports are
		// larger than the cutoff size. This guarantees that the filter is not
		// degenerate and will sum to zero.
		if (m_width_cutoff <= width_min && width_prev >= width_min) {
			cv::Range row(r, r + 1);
			cv::Mat left   = kernel(row, cv::Range(0,                             offs_both_neg - offs_line_neg));
			cv::Mat center = kernel(row, cv::Range(offs_both_neg - offs_line_neg, offs_both_neg + offs_line_pos));
			cv::Mat right  = kernel(row, cv::Range(offs_both_neg + offs_line_pos, offs_both_neg + offs_both_pos));

			double value_left   = -0.5 / left.cols;
			double value_center = +1.0 / center.cols;
			double value_right  = -0.5 / right.cols;

			left.setTo(value_left);
			center.setTo(value_center);
			right.setTo(value_right);

			offsets[r].neg = offs_both_neg;
			offsets[r].pos = offs_both_pos;
			horizon        = r;

		} else {
			return horizon + 1;
		}
		width_prev = width_min;
	}
	return 0;
}

void LineDetectionNode::PulseFilter(cv::Mat src, cv::Mat &dst, cv::Mat ker,
                                    std::vector<Offset> const &offsets,
                                    bool horizontal)
{
	ROS_ASSERT(src.type() == CV_64FC1);
	ROS_ASSERT(ker.type() == CV_64FC1);
	ROS_ASSERT(ker.rows == src.rows && ker.cols == ker.cols);
	ROS_ASSERT((int)offsets.size() == ker.rows);

	dst.create(src.rows, src.cols, CV_64FC1);
	dst.setTo(std::numeric_limits<double>::quiet_NaN());

	for (int r = m_rows - 1; r >= 0; --r) {
		Offset const &offset = offsets[r];

		// Select the pre-computed kernel for this row.
		cv::Range ker_rows(r, r + 1);
		cv::Range ker_cols(0, offset.neg + offset.pos);
		cv::Mat ker_chunk = ker(ker_rows, ker_cols);

		for (int c = m_cols - 1; c >= 0; --c) {
			// At or above the horizon line.
			if (offset.pos == 0 && offset.neg == 0) break;

			// Select the region of the source image to convolve with the
			// kernel. This may not be centered on (c, r) due to the distance
			// distortion caused by perspective projection.
			cv::Mat src_chunk;

			if (horizontal) {
				cv::Range src_rows(r, r + 1);
				cv::Range src_cols(c - offset.neg, c + offset.pos);

				if (src_cols.start >= 0 && src_cols.end <= m_cols) {
					src_chunk = src(src_rows, src_cols);
				} else {
					continue;
				}
			} else {
				cv::Range src_rows(r - offset.neg, r + offset.pos);
				cv::Range src_cols(c, c + 1);

				if (src_rows.start >= 0 && src_rows.end <= m_rows) {
					src_chunk = src(src_rows, src_cols).t();
				} else {
					continue;
				}
			}

			dst.at<double>(r, c) = src_chunk.dot(ker_chunk);
		}
	}
}

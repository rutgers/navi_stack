#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_plane/Plane.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>

#include "LineDetectionNode.hpp"

namespace line_node {

cv::Point3d VectorROStoCv(geometry_msgs::Vector3 const &vec)
{
	return cv::Point3d(vec.x, vec.y, vec.z);
}

cv::Point3d PointROStoCv(geometry_msgs::Point const &pt)
{
	return cv::Point3d(pt.x, pt.y, pt.z);
}

// nodelet conversion
LineNodelet::LineNodelet(void)
	: nh_priv("~")
{}

ros::NodeHandle &LineNodelet::getNodeHandle(void)
{
	return nh;
}

ros::NodeHandle &LineNodelet::getPrivateNodeHandle(void)
{
	return nh_priv;
}
// nodelet conversion

void LineNodelet::onInit(void)
{
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	m_it       = new it::ImageTransport(nh);
	m_valid    = false;
	m_num_prev = 0;

	nh_priv.param<bool>("debug", m_debug, false);
	nh_priv.param<int>("cutoff",    m_width_cutoff, 2);
	nh_priv.param<int>("threshold", m_threshold,    30);
	nh_priv.param<double>("border",    m_width_dead, 0.1452);
	nh_priv.param<double>("thickness", m_width_line, 0.0726);

	m_tf      = boost::make_shared<tf::TransformListener>(nh, ros::Duration(1.0));
	m_pub_pts = nh.advertise<PointCloudXYZ>("line_points", 10);

	if (m_debug) {
		ROS_WARN("debugging topics are enabled; performance may be degraded");
		m_pub_pre        = m_it->advertise("line_pre",        10);
		m_pub_distance   = m_it->advertise("line_distance",   10);
		m_pub_ker_hor    = m_it->advertise("line_kernel_hor", 10);
		m_pub_ker_ver    = m_it->advertise("line_kernel_ver", 10);
		m_pub_filter_hor = m_it->advertise("line_filter_hor", 10);
		m_pub_filter_ver = m_it->advertise("line_filter_ver", 10);
	}

	// TODO: store these subscribers in member variables
	m_sub_img   = new image_transport::SubscriberFilter(*m_it, "white", 1);
	m_sub_info  = new mf::Subscriber<CameraInfo>(nh, "camera_info", 1);
	m_sub_plane = new mf::Subscriber<Plane>(nh, "ground_plane", 1);
	m_sub = new mf::Synchronizer<Policy>(Policy(30), *m_sub_img, *m_sub_info, *m_sub_plane);
	m_sub->registerCallback(&LineNodelet::ImageCallback, this);
}

void LineNodelet::SetCutoffWidth(int width_cutoff)
{
	ROS_ASSERT(width_cutoff > 0);

	m_valid        = m_valid && (width_cutoff == m_width_cutoff);
	m_width_cutoff = width_cutoff;
}

void LineNodelet::SetDeadWidth(double width_dead)
{
	ROS_ASSERT(width_dead > 0.0);

	m_valid      = m_valid && (width_dead == m_width_dead);
	m_width_dead = width_dead;
}

void LineNodelet::SetLineWidth(double width_line)
{
	ROS_ASSERT(width_line > 0.0);

	m_valid      = m_valid && (width_line == m_width_line);
	m_width_line = width_line;
}

void LineNodelet::SetInvert(bool invert) {
	m_invert = invert;
}

void LineNodelet::SetGroundPlane(Plane plane)
{
	m_valid = m_valid && (plane.point.x == m_plane.point.x)
	                  && (plane.point.y == m_plane.point.y)
	                  && (plane.point.z == m_plane.point.z)
	                  && (plane.normal.x == m_plane.normal.x)
	                  && (plane.normal.y == m_plane.normal.y)
	                  && (plane.normal.z == m_plane.normal.z);
	m_plane = plane;
}

void LineNodelet::SetThreshold(double threshold)
{
	m_valid     = m_valid && (threshold == m_threshold);
	m_threshold = threshold;
}

void LineNodelet::SetResolution(int width, int height)
{
	ROS_ASSERT(width > 0 && height > 0);

	m_valid = m_valid && (width == m_cols) && (height == m_rows);
	m_cols  = width;
	m_rows  = height;
}

void LineNodelet::NonMaxSupr(cv::Mat src_hor, cv::Mat src_ver, PointCloudXYZ &dst)
{
	ROS_ASSERT(src_hor.rows == src_ver.rows && src_hor.cols == src_ver.cols);
	ROS_ASSERT(src_hor.type() == CV_64FC1 && src_ver.type() == CV_64FC1);
	ROS_ASSERT(m_valid);

	for (int y = 1; y < src_hor.rows - 1; ++y)
	for (int x = 1; x < src_hor.cols - 1; ++x) {
		pcl::PointXYZ pt;

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
			dst.push_back(pt);
		}
	}
}

void LineNodelet::UpdateCache(void)
{
	static cv::Point3d const dhor(1.0, 0.0, 0.0);
	static cv::Point3d const dver(0.0, 0.0, 1.0);

	ROS_ASSERT(m_width_line > 0.0);
	ROS_ASSERT(m_width_dead > 0.0);
	ROS_ASSERT(m_width_cutoff > 0);

	if (!m_valid) {
		// TODO: Switch to the actual vertical kernel.
		m_horizon_hor = GeneratePulseFilter(dhor, m_kernel_hor, m_offset_hor);
		m_horizon_ver = GeneratePulseFilter(dhor, m_kernel_ver, m_offset_ver);
		//m_horizon_ver = GeneratePulseFilter(dver, m_kernel_ver, m_offset_ver);
	}
	m_valid = true;
}

void LineNodelet::TransformPlane(Plane const &src, Plane &dst, std::string frame_id)
{
	geometry_msgs::PointStamped src_point;
	geometry_msgs::PointStamped dst_point;
	src_point.header = src.header;
	src_point.point  = src.point;

	geometry_msgs::Vector3Stamped src_normal;
	geometry_msgs::Vector3Stamped dst_normal;
	src_normal.header = src.header;
	src_normal.vector = src.normal;

	m_tf->transformPoint(frame_id, src_point, dst_point);
	m_tf->transformVector(frame_id, src_normal, dst_normal);

	dst.header = src.header;
	dst.point  = dst_point.point;
	dst.normal = dst_normal.vector;
	dst.type   = src.type;
}

void LineNodelet::ImageCallback(Image::ConstPtr const &msg_img,
                                CameraInfo::ConstPtr const &msg_cam,
                                Plane::ConstPtr const &msg_plane)
{
	namespace enc = sensor_msgs::image_encodings;

	// Transform msg_plane into the camera's coordinate frame.
	Plane plane;
	try {
		TransformPlane(*msg_plane, plane, msg_img->header.frame_id);
	} catch (tf::TransformException const &e) {
		ROS_WARN_THROTTLE(10, "%s", e.what());
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

cv::Point3d LineNodelet::GetGroundPoint(cv::Point2d pt)
{
	cv::Point3d ray     = m_model.projectPixelTo3dRay(pt);
	cv::Point3d normal = VectorROStoCv(m_plane.normal);
	cv::Point3d plane  = PointROStoCv(m_plane.point);
	return (plane.dot(normal) / ray.dot(normal)) * ray;
}

double LineNodelet::ProjectDistance(cv::Point2d pt, cv::Point3d offset)
{
	// Project the expected edge points back into the image.
	cv::Point3d P = GetGroundPoint(pt);
	cv::Point2d p1 = m_model.project3dToPixel(P);
	cv::Point2d p2 = m_model.project3dToPixel(P + offset);

	// Find the distance between the reprojected points.
	cv::Point2d diff = p2 - p1;
	return sqrt(diff.dot(diff));
}

double LineNodelet::ReprojectDistance(cv::Point2d pt, cv::Point2d offset)
{
	cv::Point3d P1 = GetGroundPoint(pt);
	cv::Point3d P2 = GetGroundPoint(pt + offset);

	cv::Point3d diff = P2 - P1;
	return sqrt(diff.dot(diff));
}

int LineNodelet::GeneratePulseFilter(cv::Point3d dw, cv::Mat &kernel, std::vector<Offset> &offsets)
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
			int a = std::max(0, offs_both_neg - offs_line_neg);
			int b = std::min(kernel.cols, offs_both_neg + offs_line_pos);
			int c = std::min(kernel.cols, offs_both_neg + offs_both_pos);

			// FIXME: Figure out why the bounds are sometimes invalid.
			bool good = 0 <= a && a < b && b < c && c < kernel.cols;
			if (!good) {
				return horizon + 1;
			}

			cv::Range row(r, r + 1);
			cv::Mat left   = kernel(row, cv::Range(0, a));
			cv::Mat center = kernel(row, cv::Range(a, b));
			cv::Mat right  = kernel(row, cv::Range(b, c));

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

void LineNodelet::PulseFilter(cv::Mat src, cv::Mat &dst, cv::Mat ker,
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
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_detection");
	line_node::LineNodelet node;
	node.onInit();
	ros::spin();
	return 0;
}

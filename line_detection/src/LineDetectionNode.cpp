#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "LineDetectionNode.hpp"

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

// TODO: Convert the direction of principal curvature to real-world coordinates.
// TODO: Compute a seperate filter kernel for the vertical direction using a
//         a vertical direction sampling vector.

LineDetectionNode::LineDetectionNode(ros::NodeHandle nh, std::string ground_id,
                                     bool debug)
	: m_debug(debug),
	  m_valid(false),
	  m_num_prev(0),
	  m_ground_id(ground_id),
	  m_nh(nh),
	  m_it(nh)
{
	m_sub_cam = m_it.subscribeCamera("image", 1, &LineDetectionNode::ImageCallback, this);
	m_pub_max = m_it.advertise("line_maxima", 10);
	m_pub_pts = m_nh.advertise<PointNormalCloud>("line_points", 10);

	if (m_debug) {
		ROS_WARN("debugging topics are enabled; performance may be degraded");

		m_pub_distance   = m_it.advertise("line_distance",   10);
		m_pub_ker_hor    = m_it.advertise("line_kernel_hor", 10);
		m_pub_ker_ver    = m_it.advertise("line_kernel_ver", 10);
		m_pub_filter_hor = m_it.advertise("line_filter_hor", 10);
		m_pub_filter_ver = m_it.advertise("line_filter_ver", 10);
		m_pub_normal     = m_it.advertise("line_normal", 10);
		m_pub_visual_one = m_nh.advertise<Marker>("/visualization_marker", 1);
		m_pub_visual     = m_nh.advertise<MarkerArray>("/visualization_marker_array", 1);
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

void LineDetectionNode::NonMaxSupr(cv::Mat src_hor, cv::Mat src_ver,
                                   std::list<cv::Point2i> &dst)
{
	ROS_ASSERT(src_hor.rows == src_ver.rows && src_hor.cols == src_ver.cols);
	ROS_ASSERT(src_hor.type() == CV_64FC1 && src_ver.type() == CV_64FC1);
	ROS_ASSERT(m_valid);

	for (int y = 1; y < src_hor.rows - 1; ++y)
	for (int x = 1; x < src_hor.cols - 1; ++x) {
		double val_hor   = src_hor.at<double>(y, x);
		double val_left  = src_hor.at<double>(y, x - 1);
		double val_right = src_hor.at<double>(y, x + 1);

		double val_ver = src_ver.at<double>(y + 0, x);
		double val_top = src_ver.at<double>(y - 1, x);
		double val_bot = src_ver.at<double>(y + 1, x);

		bool is_hor = val_hor > val_left && val_hor > val_right && val_hor > m_threshold;
		bool is_ver = val_ver > val_top  && val_ver > val_bot   && val_ver > m_threshold;

		if (is_hor || is_ver) {
			dst.push_back(cv::Point2i(x, y));
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
		ROS_INFO("ground plane P(%4f, %4f, %4f) N(%4f, %f, %f)",
			m_plane.point.x,  m_plane.point.y,  m_plane.point.z,
			m_plane.normal.x, m_plane.normal.y, m_plane.normal.z
		);

		m_horizon_hor = GeneratePulseFilter(dhor, m_kernel_hor, m_offset_hor);
		ROS_INFO("horizontal: [ %d x %d ] with horizon = %d",
			m_kernel_hor.cols, m_kernel_hor.rows, m_horizon_hor
		);

		m_horizon_ver = GeneratePulseFilter(dver, m_kernel_ver, m_offset_ver);
		ROS_INFO("vertical:   [ %d x %d ] with horizon = %d",
			m_kernel_ver.cols, m_kernel_ver.rows, m_horizon_ver
		);
	}
	m_valid = true;
}

void LineDetectionNode::ImageCallback(ImageConstPtr const &msg_img,
                                      CameraInfoConstPtr const &msg_cam)
{
	// Keep the ground plane in sync with the latest TF data.
	Plane plane;
	try {
		std::string ground_id = m_ground_id;
		std::string camera_id = msg_img->header.frame_id;

		GuessGroundPlane(m_tf, ground_id, camera_id, plane);
	} catch (tf::TransformException ex) {
		ROS_ERROR_THROTTLE(30, "%s", ex.what());
		return;
	}

	// Convert ROS messages to OpenCV data types.
	cv_bridge::CvImagePtr img_ptr;
	cv::Mat img_input;

	try {
		img_ptr = cv_bridge::toCvCopy(msg_img, image_encodings::BGR8);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR_THROTTLE(30, "%s", e.what());
		return;
	}

	// FIXME: Flush the cache when camerainfo changes.
	m_model.fromCameraInfo(msg_cam);
	img_input = img_ptr->image;

	// Update pre-computed values that were cached (only if necessary!).
	SetGroundPlane(plane);
	SetResolution(msg_img->width, msg_img->height);
	UpdateCache();

	// Processing...
	std::list<cv::Point2i> maxima;
	cv::Mat img_hor, img_ver;
	cv::Mat img_pre;

	LineColorTransform(img_input, img_pre);
	PulseFilter(img_pre, img_hor, m_kernel_hor, m_offset_hor, true);
	PulseFilter(img_pre, img_ver, m_kernel_ver, m_offset_ver, false);
	NonMaxSupr(img_hor, img_ver, maxima);

	// Publish a three-dimensional point cloud in the camera frame by converting
	// each maximum's pixel coordinates to camera coordinates using the camera's
	// intrinsics and knowledge of the ground plane.
	// TODO: Scrap the std::list middleman.
	// TODO: Do this directly in NonMaxSupr().
	// TODO: Precompute the mapping from 2D to 3D.
	PointNormalCloud::Ptr msg_pts(new PointNormalCloud);
	std::list<cv::Point2i>::iterator it;

	// Use a row vector to store unordered points (as per PCL documentation).
	msg_pts->header.stamp    = msg_img->header.stamp;
	msg_pts->header.frame_id = msg_img->header.frame_id;
	msg_pts->height = 1;
	msg_pts->width  = maxima.size();
	msg_pts->points.resize(maxima.size());

	// Calculate second-order partials to compute the Hessian matrix.
	cv::Mat dxx, dxy, dyy;
	cv::Sobel(img_pre, dxx, CV_64FC1, 2, 0);
	cv::Sobel(img_pre, dxy, CV_64FC1, 1, 1);
	cv::Sobel(img_pre, dyy, CV_64FC1, 0, 2);

	int i;
	for (it = maxima.begin(), i = 0; it != maxima.end(); ++it, ++i) {
		cv::Point2i pt_image = *it;
		cv::Point3d pt_world = GetGroundPoint(*it);

		// Calculate a vector that is normal to the direction of the line by
		// finding the principle eigenvector of the Hessian matrix centered
		// on this pixel.
		cv::Mat hessian(2, 2, CV_64FC1);
		hessian.at<double>(0, 0) = dxx.at<double>(pt_image.y, pt_image.x);
		hessian.at<double>(0, 1) = dxy.at<double>(pt_image.y, pt_image.x);
		hessian.at<double>(1, 0) = dxy.at<double>(pt_image.y, pt_image.x);
		hessian.at<double>(1, 1) = dyy.at<double>(pt_image.y, pt_image.x);

		cv::Mat eigen_vec;
		cv::Mat eigen_val;
		cv::eigen(hessian, eigen_val, eigen_vec, 0, 0);

		// Convert image coordinates to real-world coordinates on the ground plane.
		double normal_x =  eigen_vec.at<double>(0, 0);
		double normal_y = -eigen_vec.at<double>(0, 1);

		// Convert OpenCV cv::Point into a ROS geometry_msgs::Point object.
		// XXX: Normal vector is always postive; does not have the correct signs.
		pcl::PointNormal &pt = msg_pts->points[i];
		pt.x = pt_world.x;
		pt.y = pt_world.y;
		pt.z = pt_world.z;
		pt.normal[0] = 0.0;
		pt.normal[1] = ReprojectDistance(pt_image, cv::Point2d(normal_x, 0.0));
		pt.normal[2] = ReprojectDistance(pt_image, cv::Point2d(0.0, normal_y));
	}

	m_pub_pts.publish(msg_pts);


	// Two dimensional local maxima as a binary image. Detected line pixels are
	// white (255) and all other pixels are black (0).
	cv::Mat img_max(img_input.rows, img_input.cols, CV_8U, cv::Scalar(0));
	for (it = maxima.begin(); it != maxima.end(); ++it) {
		img_max.at<uint8_t>(it->y, it->x) = 255;
	}

	cv_bridge::CvImage msg_max;
	msg_max.header.stamp    = msg_img->header.stamp;
	msg_max.header.frame_id = msg_img->header.frame_id;
	msg_max.encoding = image_encodings::MONO8;
	msg_max.image    = img_max;
	m_pub_max.publish(msg_max.toImageMsg());
	if (m_debug) {
		size_t num = msg_pts->points.size();

		// Render lines every 1 m on the ground plane and render them in 3D!
		cv::Mat img_distance  = img_input.clone();
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
		msg_distance.encoding = image_encodings::RGB8;
		msg_distance.image    = img_distance;
		m_pub_distance.publish(msg_distance.toImageMsg());
		m_pub_visual_one.publish(msg_contour);

		// Visualize the matched pulse width kernels.
		cv::Mat img_ker_hor;
		cv::normalize(m_kernel_hor, img_ker_hor, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_ker_hor;
		msg_ker_hor.header.stamp    = msg_img->header.stamp;
		msg_ker_hor.header.frame_id = msg_img->header.frame_id;
		msg_ker_hor.encoding = image_encodings::MONO8;
		msg_ker_hor.image    = img_ker_hor;
		m_pub_ker_hor.publish(msg_ker_hor.toImageMsg());

		cv::Mat img_ker_ver;
		cv::normalize(m_kernel_ver, img_ker_ver, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_ker_ver;
		msg_ker_ver.header.stamp    = msg_img->header.stamp;
		msg_ker_ver.header.frame_id = msg_img->header.frame_id;
		msg_ker_ver.encoding = image_encodings::MONO8;
		msg_ker_ver.image    = img_ker_ver;
		m_pub_ker_ver.publish(msg_ker_ver.toImageMsg());

		// Visualize the raw filter responses.
		cv::Mat img_filter_hor;
		cv::normalize(img_hor, img_filter_hor, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_filter_hor;
		msg_filter_hor.header.stamp    = msg_img->header.stamp;
		msg_filter_hor.header.frame_id = msg_img->header.frame_id;
		msg_filter_hor.encoding = image_encodings::MONO8;
		msg_filter_hor.image    = img_filter_hor;
		m_pub_filter_hor.publish(msg_filter_hor.toImageMsg());

		cv::Mat img_filter_ver;
		cv::normalize(img_ver, img_filter_ver, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_filter_ver;
		msg_filter_ver.header.stamp    = msg_img->header.stamp;
		msg_filter_ver.header.frame_id = msg_img->header.frame_id;
		msg_filter_ver.encoding = image_encodings::MONO8;
		msg_filter_ver.image    = img_filter_ver;
		m_pub_filter_ver.publish(msg_filter_ver.toImageMsg());


		// Visualize normal vectors on the image. Also render the cut-off lines
		// for debugging purposes.
		cv::Mat img_normal = img_input.clone();

		int i = 0;
		for (it = maxima.begin(), i = 0; it != maxima.end(); ++it, ++i) {
			double normal_x = msg_pts->points[i].normal[1];
			double normal_y = msg_pts->points[i].normal[2];

			cv::Point2d point(it->x, it->y);
			cv::Point2d normal(normal_x, normal_y);
			double scale = 25 / sqrt(pow(normal_x, 2) + pow(normal_y, 2));
			normal = normal * scale;

			cv::line(img_normal, point, point + normal, cv::Scalar(255, 0, 0));
		}

		cv::Point pt_hor0(0,               m_horizon_hor);
		cv::Point pt_hor1(img_normal.cols, m_horizon_hor);
		cv::line(img_normal, pt_hor0, pt_hor1, cv::Scalar(255, 0, 0));

		cv::Point pt_ver0(0,               m_horizon_ver);
		cv::Point pt_ver1(img_normal.cols, m_horizon_ver);
		cv::line(img_normal, pt_ver0, pt_ver1, cv::Scalar(255, 0, 0));

		cv_bridge::CvImage msg_normal;
		msg_normal.header.stamp    = msg_img->header.stamp;
		msg_normal.header.frame_id = msg_img->header.frame_id;
		msg_normal.encoding = image_encodings::RGB8;
		msg_normal.image    = img_normal;
		m_pub_normal.publish(msg_normal.toImageMsg());

		// Render normal vectors as lines that can be visualized in RViz.
		// TODO: Fetch .ns from the current node's __name.
		// TODO: Extract most of this to a helper function.
		MarkerArray msg_visual;
		msg_visual.markers.resize(MAX(num, m_num_prev));

		for (size_t i = 0; i < num; ++i) {
			Marker &marker = msg_visual.markers[i];
			marker.header.stamp    = msg_img->header.stamp;
			marker.header.frame_id = msg_img->header.frame_id;
			marker.ns     = "line_normal";
			marker.id     = i;
			marker.type   = Marker::ARROW;
			marker.action = Marker::ADD;

			// Tail of the vector is on the detected point.
			geometry_msgs::Point &pos = marker.pose.position;
			pos.x = msg_pts->points[i].x;
			pos.y = msg_pts->points[i].y;
			pos.z = msg_pts->points[i].z;

			// TODO: Are the signs/order correct?
			double normal_x = msg_pts->points[i].normal[1];
			double normal_y = msg_pts->points[i].normal[2];
			double yaw = -atan2(normal_y, normal_x);
			marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, yaw, 0.0);

			geometry_msgs::Vector3 &scale = marker.scale;
			scale.x = 0.5;
			scale.y = 1.0;
			scale.z = 1.0;

			std_msgs::ColorRGBA &color = marker.color;
			color.r = 1.0;
			color.g = 0.0;
			color.b = 0.0;
			color.a = 1.0;
		}

		// Clear the old markers if they were not already modified. Old markers
		// will linger until they are modified without this hack.
		for (size_t i = num; i < m_num_prev; ++i) {
			Marker &marker = msg_visual.markers[i];
			marker.header.stamp    = msg_img->header.stamp;
			marker.header.frame_id = msg_img->header.frame_id;
			marker.ns     = "line_normal";
			marker.id     = i;
			marker.action = Marker::DELETE;
		}

		m_pub_visual.publish(msg_visual);
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

			// TODO: Clip these ranges at the image's borders.
			cv::Mat left   = kernel(row, cv::Range(0,                             offs_both_neg - offs_line_neg));
			cv::Mat center = kernel(row, cv::Range(offs_both_neg - offs_line_neg, offs_both_neg + offs_line_pos));
			cv::Mat right  = kernel(row, cv::Range(offs_both_neg + offs_line_pos, offs_both_neg + offs_both_pos));

			double value_left   = left.cols  * -1.0 / (left.cols + right.cols);
			double value_center = +1.0 / center.cols;
			double value_right  = right.cols * -1.0 / (left.cols + right.cols);

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

	// TODO: Dynamically select the value of invalid regions in the image.
	dst.create(src.rows, src.cols, CV_64FC1);
	dst.setTo(-255);

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

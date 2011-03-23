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

		m_pub_kernel = m_it.advertise("line_kernel", 10);
		m_pub_normal = m_it.advertise("line_normal", 10);
		m_pub_visual = m_nh.advertise<MarkerArray>("/visualization_marker_array", 1);
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

void LineDetectionNode::SetIntrinsics(cv::Mat mint)
{
	ROS_ASSERT(mint.rows == 3 && mint.cols == 3);
	ROS_ASSERT(mint.type() == CV_64FC1);

	for (int r = 0; r < mint.rows; ++r)
	for (int c = 0; c < mint.cols; ++c) {
		m_valid = m_valid && mint.at<double>(r, c) == m_mint.at<double>(r, c);
	}

	if (!m_valid) {
		mint.copyTo(m_mint);
	}
}

void LineDetectionNode::SetGroundPlane(Plane plane)
{
	m_valid = m_valid && (plane.point.x == m_plane.point.x)
	                  && (plane.point.y == m_plane.point.y)
	                  && (plane.point.z == m_plane.point.z)
	                  && (plane.normal.x == m_plane.normal.x)
	                  && (plane.normal.y == m_plane.normal.y)
	                  && (plane.normal.z == m_plane.normal.z);
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

void LineDetectionNode::MatchedFilter(cv::Mat src, cv::Mat &dst_hor,
                                      cv::Mat &dst_ver)
{
	ROS_ASSERT(src.type() == CV_64FC1);
	ROS_ASSERT(m_valid);

	cv::Mat ker_row, ker_col;
	dst_hor.create(src.rows, src.cols, CV_64FC1);
	dst_ver.create(src.rows, src.cols, CV_64FC1);

	// TODO: Change this to an invalid value (maybe NaN or -INFINITY?).
	dst_hor.setTo(-255.0);
	dst_ver.setTo(-255.0);

	for (int r = m_cutoff; r < m_rows; ++r)
	for (int c = 0; c < m_cols; ++c) {
		int kernel_size = m_cache_line[r] + 2 * m_cache_dead[r];
		int left   = c - (kernel_size + 0) / 2;
		int right  = c + (kernel_size + 1) / 2;
		int top    = r - (kernel_size + 0) / 2;
		int bottom = r + (kernel_size + 1) / 2;

		cv::Range range_row(r, r + 1);
		cv::Range range_col(c, c + 1);

		cv::Mat kernel = m_cache_kernel(range_row, cv::Range(0, kernel_size));

		// TODO: Shrink the filter instead of ignoring these tricky cases.
		// TODO: Manually perform the vertical dot product if necessary.
		if (left >= 0 && r >= 0 && right < m_cols && (r + 1 < m_rows)) {
			cv::Mat src_hor = src(range_row,  cv::Range(left, right));
			dst_hor.at<double>(r, c) = kernel.dot(src_hor);
		}

		if (c >= 0 && top >= 0 && c + 1 < m_cols && bottom < m_rows) {
			cv::Mat src_ver = src(cv::Range(top, bottom), range_col);
			dst_ver.at<double>(r, c) = kernel.dot(src_ver.t());
		}
	}
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
	ROS_ASSERT(m_width_line > 0.0);
	ROS_ASSERT(m_width_dead > 0.0);
	ROS_ASSERT(m_width_cutoff > 0);
	ROS_ASSERT(m_mint.rows == 3 && m_mint.cols == 3);
	ROS_ASSERT(m_mint.type() == CV_64FC1);

	if (m_valid) return;

	ROS_INFO("rebuilding cache with changed parameters");

	m_cache_line.resize(m_rows);
	m_cache_dead.resize(m_rows);

	m_cache_kernel.create(m_rows, m_cols, CV_64FC1);
	m_cache_kernel.setTo(0.0);

	// Pre-compute the widths necessary to construct the matched pulse-width
	// filter. Assume distances to not change along rows in the image (i.e. the
	// image is rectified).
	int prev_line = INT_MAX;
	int prev_dead = INT_MAX;

	// Just in case we never see the horizon.
	m_horizon = 0;
	m_cutoff  = 0;

	for (int r = m_rows - 1; r >= 0; --r) {
		// TODO: Calculate separate distances for row and column filters.
		// TODO: Use a Taylor approximation to simplify the width calculation.
		cv::Point2d middle(m_cols / 2, r);
		cv::Point3d delta_line(m_width_line, 0.0, 0.0);
		cv::Point3d delta_dead(m_width_dead, 0.0, 0.0);

		m_cache_line[r] = GetDistSize(middle, delta_line, m_mint, m_plane);
		m_cache_dead[r] = GetDistSize(middle, delta_dead, m_mint, m_plane);

		// Stop processing when the line is too small to effectively filter. Also
		// estimate the horizon line by finding where the ray from the camera
		// is parallel to the ground (i.e. there is no intersection point).
		bool wrap = m_cache_line[r] > prev_line || m_cache_dead[r] > prev_dead;

		if (m_cutoff == 0 && m_cache_line[r] < m_width_cutoff) {
			m_cutoff = r + 1;
		}

		if (wrap || m_cache_line[r] < 1.0) {
			m_horizon = r + 1;
			break;
		}

		// Pre-compute and cache a small filter kernel.
		cv::Mat kernel = m_cache_kernel.row(r);
		BuildLineFilter(kernel, 0, m_cols, m_cache_line[r], m_cache_dead[r], true);

		prev_line   = m_cache_line[r];
		prev_dead   = m_cache_dead[r];
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
	cv::Mat mint;

	try {
		img_ptr = cv_bridge::toCvCopy(msg_img, image_encodings::BGR8);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR_THROTTLE(30, "%s", e.what());
		return;
	}

	img_input = img_ptr->image;
	CameraInfoToMat(msg_cam, mint);

	// Update pre-computed values that were cached (only if necessary!).
	SetIntrinsics(mint);
	SetGroundPlane(plane);
	SetResolution(msg_img->width, msg_img->height);
	UpdateCache();

	// Processing...
	std::list<cv::Point2i> maxima;
	cv::Mat img_hor, img_ver;
	cv::Mat img_pre;

	LineColorTransform(img_input, img_pre);
	MatchedFilter(img_pre, img_hor, img_ver);
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
		cv::Point2i pt_image(*it);
		cv::Point3d ray, pt_world;

		GetPixelRay(mint, pt_image, ray);
		GetRayPlaneInt(ray, plane, pt_world);

		// Calculate a vector that is normal to the direction of the line by
		// finding the principle eigenvector of the Hessian matrix centered
		// on this pixel.
		// TODO: Convert distances in the image to distances in real life. I am
		//       not sure if I should do this before or after finding the
		//       Hessian matrix.
		cv::Mat hessian(2, 2, CV_64FC1);
		hessian.at<double>(0, 0) = dxx.at<double>(it->y, it->x);
		hessian.at<double>(0, 1) = dxy.at<double>(it->y, it->x);
		hessian.at<double>(1, 0) = dxy.at<double>(it->y, it->x);
		hessian.at<double>(1, 1) = dyy.at<double>(it->y, it->x);

		cv::Mat eigen_vec;
		cv::Mat eigen_val;
		cv::eigen(hessian, eigen_val, eigen_vec); //, 0, 0);

		// Convert image coordinates to real-world coordinates on the ground
		// plane. This is especially important since a small change in 
		double normal_x =  eigen_vec.at<double>(0, 0);
		double normal_y = -eigen_vec.at<double>(0, 1);

		// TODO: Replace this hack with a Taylor approximation.
		cv::Point2d dx(normal_x, 0);
		cv::Point2d dy(0, normal_y);
		double ground_x = GetPixSize(*it, dx, m_mint, m_plane);
		double ground_y = GetPixSize(*it, dy, m_mint, m_plane);

		// Convert OpenCV cv::Point into a ROS geometry_msgs::Point object.
		pcl::PointNormal &pt = msg_pts->points[i];
		pt.x = pt_world.x;
		pt.y = pt_world.y;
		pt.z = pt_world.z;
		pt.normal[0] = 0.0;
		pt.normal[1] = ground_x;
		pt.normal[2] = ground_y;
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

		// Visualize the matched pulse width kernel.
		cv::Mat img_kernel;
		cv::normalize(m_cache_kernel, img_kernel, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_kernel;
		msg_kernel.header.stamp    = msg_img->header.stamp;
		msg_kernel.header.frame_id = msg_img->header.frame_id;
		msg_kernel.encoding = image_encodings::MONO8;
		msg_kernel.image    = img_kernel;
		m_pub_kernel.publish(msg_kernel.toImageMsg());

		// Visualize normal vectors on the image. Also render the horizon and
		// cut-off lines for debugging purposes.
		cv::Mat img_normal = img_input.clone();

		for (it = maxima.begin(), i = 0; it != maxima.end(); ++it, ++i) {
			double normal_x = msg_pts->points[i].normal[1];
			double normal_y = msg_pts->points[i].normal[2];

			cv::Point2d point(it->x, it->y);
			cv::Point2d normal(normal_x, normal_y);
			double scale = 25 / sqrt(pow(normal_x, 2) + pow(normal_y, 2));
			normal = normal * scale;

			cv::line(img_normal, point, point + normal, cv::Scalar(255, 0, 0));
		}

		std::cout << "horizon = " << m_horizon << ", cutoff = " << m_cutoff << std::endl;

		cv::Point2d pt_hor_l(0.0, m_horizon), pt_hor_r(img_input.cols, m_horizon);
		cv::Point2d pt_cut_l(0.0, m_cutoff),  pt_cut_r(img_input.cols, m_cutoff);
		cv::line(img_normal, pt_hor_l, pt_hor_r, cv::Scalar(255, 0, 0));
		cv::line(img_normal, pt_cut_l, pt_cut_r, cv::Scalar(0, 255, 0));

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
			marker.ns     = "line_detection";
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
			marker.ns     = "line_detection";
			marker.id     = i;
			marker.action = Marker::DELETE;
		}

		m_pub_visual.publish(msg_visual);
		m_num_prev = num;
	}
}

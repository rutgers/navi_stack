#include "LineDetectionNode.hpp"

LineDetectionNode::LineDetectionNode(ros::NodeHandle nh, std::string ground_id,
                                     bool debug)
	: m_debug(debug),
	  m_valid(false),
	  m_width_cutoff(3), // TODO: Make this a parameter.
	  m_ground_id(ground_id),
	  m_nh(nh),
	  m_it(nh)
{
	m_sub_cam = m_it.subscribeCamera("image", 1, &LineDetectionNode::ImageCallback, this);
	m_pub_pts = m_nh.advertise<sensor_msgs::PointCloud>("line_points", 10);

	if (m_debug) {
		ROS_WARN("debugging topics are enabled; performance may be degraded");

		m_pub_kernel = m_it.advertise("line_kernel", 10);
	}
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
		cv::Point2d middle(m_cols / 2, r);
		m_cache_line[r] = GetDistSize(middle, m_width_line, m_mint, m_plane);
		m_cache_dead[r] = GetDistSize(middle, m_width_dead, m_mint, m_plane);

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
	// TODO: Do this directly in NonMaxSupr().
	// TODO: Precompute the mapping from 2D to 3D.
	sensor_msgs::PointCloud msg_pts;
	std::list<cv::Point2i>::const_iterator it;

	for (it = maxima.begin(); it != maxima.end(); ++it) {
		cv::Point2i pt_image(*it);
		cv::Point3d ray, pt_world;

		GetPixelRay(mint, pt_image, ray);
		GetRayPlaneInt(ray, plane, pt_world);

		// Convert OpenCV cv::Point into a ROS geometry_msgs::Point object.
		geometry_msgs::Point32 msg_pt;
		msg_pt.x = (float)pt_world.x;
		msg_pt.y = (float)pt_world.y;
		msg_pt.z = (float)pt_world.z;
		msg_pts.points.push_back(msg_pt);
	}

	msg_pts.header.stamp    = msg_img->header.stamp;
	msg_pts.header.frame_id = msg_img->header.frame_id;
	m_pub_pts.publish(msg_pts);

	if (m_debug) {
		// Visualize the matched pulse width kernel.
		cv::Mat img_kernel;
		RenderKernel(img_kernel);

		cv::Mat img_kernel_8u;
		cv::normalize(img_kernel, img_kernel_8u, 0, 255, CV_MINMAX, CV_8UC1);

		cv_bridge::CvImage msg_kernel;
		msg_kernel.header.stamp    = msg_img->header.stamp;
		msg_kernel.header.frame_id = msg_img->header.frame_id;
		msg_kernel.encoding = image_encodings::MONO8;
		msg_kernel.image    = img_kernel_8u;
		m_pub_kernel.publish(msg_kernel.toImageMsg());
	}
}

void LineDetectionNode::RenderKernel(cv::Mat &dst)
{
	m_cache_kernel.copyTo(dst);

#if 0
	for (int r = m_horizon; r < m_rows; ++r) {
#if 0
		double width_line = m_cache_line[r];
		double width_dead = m_cache_dead[r];

		cv::Mat row = dst.row(r);
		BuildLineFilter(row, m_cols / 2, m_cols, width_line, width_dead, false);
#else
		cv::Mat kernel = m_cache_kernel[r];
		int     size   = kernel.cols;

		int left   = m_cols / 2 - (size + 0) / 2;
		int right  = m_cols / 2 + (size + 1) / 2;

		if (left >= 0 && r >= 0 && right < m_cols && (r + 1 < m_rows)) {
			cv::Mat chunk = dst(cv::Range(r, r + 1), cv::Range(left, right));
			kernel.copyTo(chunk);
		}
	}
#endif
#endif
}

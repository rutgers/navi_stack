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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

#define DEBUG

namespace image_encodings = sensor_msgs::image_encodings;

using geometry_msgs::PointStamped;
using geometry_msgs::Vector3Stamped;
using image_transport::CameraSubscriber;
using sensor_msgs::CameraInfoConstPtr;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

struct Plane {
	cv::Point3d normal;
	cv::Point3d point;
};

static CameraSubscriber sub_cam;
static ros::Publisher   pub_pts;
static tf::TransformListener *listener;

#ifdef DEBUG
static image_transport::Publisher pub_debug;
#endif

static std::string p_frame;
static double p_thickness;
static double p_threshold;

/**
 * Solves for the ray that starts at the camera center and passes through a
 * given pixel on the image plane. Output is expressed with respect to the
 * camera coordinate frame.
 *
 * \param intrinsic camera parameters; obtained by camera calibration
 * \param pt        point expressed in the image frame
 * \param dst       output ray stored as a column vector
 */
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

/**
 * Solves for the intersection of a line and a plane. This assumes that the
 * line passes through the origin.
 *
 * Assume the plane is parameterized by an arbitrary point on its surface,
 * \f$ p_p \f$, and a normal vector \f$ n \f$. Similarly, assume the line is
 * parameterized by an arbitrary point, \f$ p_l \f$, and a direction vector,
 * $ \f$ v \f$. The intersection point, \f$ p \f$, must satisfy
 * \f{eqnarray*}{
 *     (p - p_p) \cdot n    & = & 0 \\
 *     -p + p_l + \lambda v & = & 0,
 * \f}
 * where \f$ \lambda \f$ is a scalar that uniquely defines the intersection
 * point.
 *
 * Solving for \f$ \lambda \f$ results in a closed-form expression containing
 * only known parameters. Substituting this value into the equation of the
 * line yields an intersection point of
 * \f[
 *     p = p_l + \left(\frac{n \cdot (p_p - p_l)}{n \cdot v}\right) v.
 * \f]
 *
 * \param ray   arbitrary point on the line
 * \param plane plane parameterized as a point and a normal vector
 * \param dst   output intersection point
 */
void GetRayPlaneInt(cv::Point3d ray, Plane plane, cv::Point3d &dst)
{
	dst = (plane.normal.dot(plane.point) / plane.normal.dot(ray)) * ray;
}

/**
 * Calculates the pixel dimensions of a real-world distance at a specific
 * point in the image. This function assumes that all points in the image
 * reside on a known plane expressed in the camera coordinate frame. Output
 * will be in the same dimensions as were used for camera calibration.
 *
 * \param pt    reference point in the image
 * \param dist  distance in real-world units
 * \param mint  intrinsic camera parameters
 * \param plane expressed in the camera coordinate frame
 * \return corresponding distance in image coordinates
 */
double GetDistSize(cv::Point2d pt0, double dist, cv::Mat mint, Plane plane)
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
	pt1_world.at<double>(0, 0) = pt0_world.x + dist;
	pt1_world.at<double>(1, 0) = pt0_world.y;
	pt1_world.at<double>(2, 0) = pt0_world.z;
	cv::Mat pt1_mat = mint * pt1_world;

	// Project this offset point into the image.
	cv::Point2d pt1(pt1_mat.at<double>(0, 0), pt1_mat.at<double>(1, 0));
	pt1 *= 1.0 / pt1_mat.at<double>(2, 0);

	// Find the distance from the offset point to the original point.
	cv::Point2d offset = pt1 - pt0;
	return sqrt(offset.dot(offset));
}

/**
 * Extracts the intrinsic matrix from a ROS CameraInfo message into an OpenCV
 * matrix object. This should be replaced by an operation in CvBridge, if
 * equivalent functionality is added.
*
 * \param msg  CameraInfo message
 * \param mint output intrinsic camera parameters
 */
void CameraInfoToMat(CameraInfoConstPtr const &msg, cv::Mat &mint)
{
	mint.create(3, 3, CV_64FC1);

	for (int i = 0; i < 3; ++i)
	for (int j = 0; j < 3; ++j) {
		mint.at<double>(i, j) = msg->K[i * 3 + j];
	}
}

/**
 * Construct a pulse-width filter tailored to the desired width with the
 * following shape:
 *
 *        /  0 : 0          <= x < c - w
 *        | -1 : c - (3/2)w <= x < c - w/2
 * f(x) = | +1 : c - (1/2)w <= x < c + w/2
 *        | -1 : c + (1/2)w <= x < c + w
 *        \  0 : c + (3/2)w <= x,
 *
 * where $c$ is the filter's center and $w$ is its pulse width. Graphically,
 * this looks like:
 *
 *           +------+           <= +1.0
 *           |      |
 *           |      |
 * ---+      |      |      +--- <=  0.0
 *    |      |      |      |
 *    +------+      +------+    <= -0.5
 *
 * Note that the filter is constructed such that it will have a zero response
 * on regions of solid color (i.e. the kernel elements sum to zero).
 *
 * \param x     filter center
 * \param dim   width of the image to be filtered, in pixels
 * \param width filter width
 * \param ker   output parameter for the kernel
 */
void BuildLineFilter(int x, int dim, int width, cv::Mat &ker)
{
	ROS_ASSERT(0 <= x && x < dim);
	ROS_ASSERT(dim > 0);

	int x1 = MAX(x - width * 3 / 2, 0);       // falling edge, trough
	int x2 = MAX(x - width * 1 / 2, 0);       // rising edge,  peak
	int x3 = MIN(x + width * 1 / 2, dim - 1); // falling edge, peak
	int x4 = MIN(x + width * 3 / 2, dim - 1); // rising edge,  trough

	ker.create(1, dim, CV_64FC1);
	ker.setTo(0.0);

	cv::Mat lo_l = ker(cv::Range(0, 1), cv::Range(x1, x2));
	cv::Mat lo_r = ker(cv::Range(0, 1), cv::Range(x3, x4));
	cv::Mat hi   = ker(cv::Range(0, 1), cv::Range(x2, x3));

	lo_l.setTo(-0.5 / lo_l.cols);
	lo_r.setTo(-0.5 / lo_r.cols);
	hi.setTo(  +1.0 / hi.cols);
}

/**
 * Filter a grayscale image along each row and column using a matched
 * pulse-width filter centered at each pixel. See BuildLineFilter() for an
 * in-depth description of the pulse-width filter.
 *
 * \param src input grayscale image
 * \param dst_hor   horizontally filtered output image
 * \param dst_ver   vertically filtered output image
 * \param mint      intrinsic camera matrix
 * \param plane     ground plane in the camera's coordinate frame
 * \param thick     line thickness in real-world coordinates
 */
void LineFilter(cv::Mat src, cv::Mat &dst_hor, cv::Mat &dst_ver, cv::Mat mint,
                Plane plane, double thick)
{
	ROS_ASSERT(mint.rows == 3 && mint.cols == 3);
	ROS_ASSERT(mint.type() == CV_64FC1);
	ROS_ASSERT(src.type() == CV_64FC1);
	ROS_ASSERT(thick > 0);

	cv::Mat ker_row, ker_col;

	dst_hor.create(src.rows, src.cols, CV_64FC1);
	dst_ver.create(src.rows, src.cols, CV_64FC1);
	dst_hor.setTo(0);
	dst_ver.setTo(0);

	double width      = 0.0;
	double width_prev = INFINITY;

	for (int y = src.rows - 1; y >= 0; --y)
	for (int x = src.cols - 1; x >= 0; --x) {
		width = GetDistSize(cv::Point2d(x, y), thick, mint, plane);

		// Stop processing at the horizon.
		if (width > width_prev) return;
		width_prev = width;

		cv::Mat row = src.row(y);
		cv::Mat col = src.col(x);

		BuildLineFilter(x, src.cols, width, ker_row);
		dst_hor.at<double>(y, x) = ker_row.dot(row);

		BuildLineFilter(y, src.rows, width, ker_col);
		dst_ver.at<double>(y, x) = ker_col.t().dot(col);
	}
	}
}

/**
 * Perform non-maximal supression to reduce the amount of superfluous
 * information in a grayscale image. This simulaneously performs non-maximal
 * supression on horizontally and vertically filtered images to avoid duplicate
 * maxima. Designed to directly accept the output of LineFilter() with no
 * modifications.
 *
 * \param src_hor   horizontally filtered image
 * \param src_ver   vertically filtered image
 * \param dst       output parameter; list of local maxima
 * \param threshold minimum value acceptable for a maximum
 */
void FindMaxima(cv::Mat src_hor, cv::Mat src_ver, std::list<cv::Point2i> &dst,
                double threshold)
{
	ROS_ASSERT(src_hor.rows == src_ver.rows && src_hor.cols == src_ver.cols);
	ROS_ASSERT(src_hor.type() == CV_64FC1 && src_ver.type() == CV_64FC1);

	for (int y = 1; y < src_hor.rows - 1; ++y)
	for (int x = 1; x < src_hor.cols - 1; ++x) {
		uint8_t val_hor = src_hor.at<uint8_t>(y, x);
		uint8_t val_ver = src_hor.at<uint8_t>(y, x);
		uint8_t val_l = src_hor.at<uint8_t>(y, x - 1);
		uint8_t val_r = src_hor.at<uint8_t>(y, x + 1);
		uint8_t val_t = src_ver.at<uint8_t>(y - 1, x);
		uint8_t val_b = src_ver.at<uint8_t>(y + 1, x);

		bool is_hor = val_hor > val_l && val_hor > val_r && val_hor > threshold;
		bool is_ver = val_ver > val_t && val_ver > val_b && val_ver > threshold;

		if (is_hor || is_ver) {
			dst.push_back(cv::Point2i(x, y));
		}
	}
}

/**
 * Transform a color 8-bit BGR image into a grayscale image where regions of
 * white are over-emphasized. Note that the input of this function is of type
 * CV_8U and the output it CV_64F (for future computations).
 * \param src color input image, 8-bit BGR
 * \param dst grayscale output image, 64-bit
 */
void LineColorTransform(cv::Mat src, cv::Mat &dst)
{
	ROS_ASSERT(src.type() == CV_8UC3);

	// Convert to the HSV color space to get saturation and intensity.
	std::vector<cv::Mat> img_chan;
	cv::Mat img_hsv;
	cv::cvtColor(src, img_hsv, CV_BGR2HSV);
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
	LineFilter(img_white, img_hor, img_ver, mint, plane, p_thickness);
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
	// Calculate the expected width of a line in each row of the image. Stop
	// at the horizon line by detecting an increase in pixel width.
	double width_old = INFINITY;
	double width_new = 0.0;

	cv::Mat img_debug = img_input.clone();

	for (int y = img_debug.rows - 1; y >= 0 && width_new <= width_old; --y) {
		cv::Point2d mid(img_debug.cols / 2.0, y);
		width_new = GetDistSize(mid, p_thickness, mint, plane);
		if (width_new >= width_old) break;
		width_old = width_new;

		cv::Point2d mid_left  = mid - cv::Point2d(width_new / 2.0, 0.0);
		cv::Point2d mid_right = mid + cv::Point2d(width_new / 2.0, 0.0);

		// Render a simulated line on the image for debugging purposes.
		cv::Point2d offset(2, 0);
		cv::line(img_debug, mid_left - offset,  mid_left + offset,  cv::Scalar(0, 0, 255));
		cv::line(img_debug, mid_right - offset, mid_right + offset, cv::Scalar(0, 0, 255));
	}

	cv_bridge::CvImage msg_debug;
	msg_debug.header.stamp    = msg_img->header.stamp;
	msg_debug.header.frame_id = msg_img->header.frame_id;
	msg_debug.encoding = image_encodings::BGR8;
	msg_debug.image    = img_debug;
	pub_debug.publish(msg_debug.toImageMsg());
#endif
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_detection");
	ros::NodeHandle nh;

	listener = new tf::TransformListener;

	nh.param<double>("thickness", p_thickness, 0.0726);
	nh.param<double>("threshold", p_threshold, 0.0400);
	nh.param<std::string>("frame", p_frame, "base_footprint");

	image_transport::ImageTransport it(nh);
	sub_cam = it.subscribeCamera("image", 1, &callback);
	pub_pts = nh.advertise<sensor_msgs::PointCloud>("line_points", 10);

#ifdef DEBUG
	pub_debug = it.advertise("line_debug", 10);
#endif

	ros::spin();
	return 0;
}

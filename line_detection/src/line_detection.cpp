#include <cmath>
#include <vector>

#include <opencv/cv.h>

#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Point32.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>

using image_transport::CameraSubscriber;
using sensor_msgs::CameraInfoConstPtr;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

struct Plane {
	cv::Point3d normal;
	cv::Point3d point;
};

static sensor_msgs::CvBridge bridge;

static CameraSubscriber sub_cam;
static ros::Publisher   pub_pts;
static ros::Publisher   pub_debug;

static double p_thickness;
static double p_threshold;
static Plane  p_plane;

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

void BuildLineFilter(int x, int dim, int width, cv::Mat &ker)
{
	int x1 = x - width;     // falling edge, trough
	int x2 = x - width / 2; // rising edge,  peak
	int x3 = x + width / 2; // falling edge, peak
	int x4 = x + width;     // rising edge,  trough

	ker.create(dim, 1, CV_64F);
	ker.setTo(0);

	// Neglect pixels where the kernel hits the edge of the image.
	if (x1 >= 0 && x4 < dim) {
		cv::Mat lo_l = ker(cv::Range(x1, x2), cv::Range(0, 1));
		cv::Mat lo_r = ker(cv::Range(x3, x4), cv::Range(0, 1));
		cv::Mat hi   = ker(cv::Range(x2, x3), cv::Range(0, 1));

		lo_l.setTo(-1.0);
		lo_r.setTo(-1.0);
		hi.setTo(+1.0);
	}
}

void callback(ImageConstPtr const &msg_img, CameraInfoConstPtr const &msg_cam)
{
	// Convert ROS message formats to OpenCV data types.
	cv::Mat img = bridge.imgMsgToCv(msg_img, "bgr8");
	cv::Mat mint;
	CameraInfoToMat(msg_cam, mint);

	// Calculate the expected width of a line in each row of the image. Stop
	// at the horizon line by detecting an increase in pixel width.
	std::vector<double> line_width(img.rows, 0.0);
	double width_old = INFINITY;
	double width_new = 0.0;

	for (int y = img.rows - 1; y >= 0 && width_new <= width_old; --y) {
		cv::Point2d mid(img.cols / 2.0, y);
		width_new = GetDistSize(mid, p_thickness, mint, p_plane);
		if (width_new >= width_old) break;

		line_width[y] = width_new;
		width_old     = width_new;

		cv::Point2d mid_left  = mid - cv::Point2d(width_new / 2.0, 0.0);
		cv::Point2d mid_right = mid + cv::Point2d(width_new / 2.0, 0.0);

		// Render a simulated line on the image for debugging purposes.
		cv::Point2d offset(2, 0);
		cv::line(img, mid_left - offset,  mid_left + offset,  cv::Scalar(0, 0, 255));
		cv::line(img, mid_right - offset, mid_right + offset, cv::Scalar(0, 0, 255));
	}

	IplImage img_old = img;
	sensor_msgs::Image msg_out = *bridge.cvToImgMsg(&img_old);
	msg_out.header.stamp       = msg_img->header.stamp;
	msg_out.header.frame_id    = msg_img->header.frame_id;

	// Convert the image into the HSV color space. White lines should have a
	// relatively low saturation and a relatively high brightness.
	// TODO: Find a better way of merging saturation and brightness.
	std::vector<cv::Mat> img_chan;
	cv::Mat img_hsv;
	cv::cvtColor(img, img_hsv, CV_BGR2HSV);
	cv::split(img_hsv, img_chan);

	cv::Mat img_sat = 255 - img_chan[1];
	cv::Mat img_val = img_chan[2];
	cv::Mat white;
	cv::min(img_sat, img_val, white);

	// Create a filter with the below shape:
	//
	//        +------+        <= +v
	//        |      |
	// ---+   |      |   +--- <=  0
	//    |   |      |   |
	//    +---+      +---+    <= -v
	//
	// The peak is the same width as the combination of the two equal-sized
	// throughs. This is normalized to be zero-centered (i.e. sum to zero).
	cv::Mat img_hor(img.rows, img.cols, CV_8U);
	cv::Mat img_ver(img.rows, img.cols, CV_8U);

	for (int y = 0; y < img.rows; ++y)
	for (int x = 0; x < img.rows; ++x) {
		double width = GetDistSize(cv::Point2d(x, y), p_thickness, mint, p_plane);

		cv::Mat ker_row, ker_col;
		BuildLineFilter(x, img.cols, width, ker_row);
		BuildLineFilter(y, img.rows, width, ker_col);

		img_ver.at<uint8_t>(y, x) = (uint8_t)ker_row.dot(img.row(y));
		img_ver.at<uint8_t>(y, x) = (uint8_t)ker_col.dot(img.col(x));
	}

	// Perform non-maximal supression in the same direction as the matched pulse-
	// width filter, greatly reducing the amount of superflous information in the
	// data by removing clumps.
	std::list<cv::Point2i> maxima;

	for (int y = 1; y < img.rows - 1; ++y)
	for (int x = 1; x < img.cols - 1; ++x) {
		uint8_t val_m = img_hor.at<uint8_t>(y, x);
		uint8_t val_l = img_hor.at<uint8_t>(y, x - 1);
		uint8_t val_r = img_hor.at<uint8_t>(y, x + 1);
		uint8_t val_t = img_ver.at<uint8_t>(y - 1, x);
		uint8_t val_b = img_ver.at<uint8_t>(y + 1, x);

		bool is_threshold = val_m > p_threshold;
		bool is_hor_max   = val_m > val_l && val_m > val_r;
		bool is_ver_max   = val_m > val_t && val_m > val_b;

		if (is_threshold || is_hor_max || is_ver_max) {
			maxima.push_back(cv::Point2i(x, y));
		}
	}

	// Publish a three-dimensional point cloud in the camera frame by converting
	// each maximum's pixel coordinates to camera coordinates using the camera's
	// intrinsics and knowledge of the ground plane.
	sensor_msgs::PointCloud pts_msg;
	pts_msg.header.stamp    = msg_img->header.stamp;
	pts_msg.header.frame_id = msg_img->header.frame_id;

	std::list<cv::Point2i>::const_iterator it;

	for (it = maxima.begin(); it != maxima.end(); ++it) {
		cv::Point2i pt_image(*it);
		cv::Point3d ray, pt_world;

		GetPixelRay(mint, pt_image, ray);
		GetRayPlaneInt(ray, p_plane, pt_world);

		// Convert OpenCV cv::Point into a ROS geometry_msgs::Point object.
		geometry_msgs::Point32 pt_msg;
		pt_msg.x = (float)pt_world.x;
		pt_msg.y = (float)pt_world.y;
		pt_msg.z = (float)pt_world.z;
		pts_msg.points.push_back(pt_msg);
	}

	pub_debug.publish(msg_out);
	pub_pts.publish(pts_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_detection");
	ros::NodeHandle nh;

	// TODO: Populate p_plane from either messages or parameters.
	nh.param("thickness", p_thickness, 0.0726);
	nh.param("threshold", p_threshold, 0.0400);

	image_transport::ImageTransport it(nh);
	sub_cam = it.subscribeCamera("image", 1, &callback);
	pub_pts = nh.advertise<sensor_msgs::PointCloud>("line_points", 10);

	ros::spin();
	return 0;
}

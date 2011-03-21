#ifndef UTIL_H_
#define UTIL_H_

#include <list>
#include <opencv/cv.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

using sensor_msgs::CameraInfoConstPtr;
using sensor_msgs::ImageConstPtr;

struct Plane {
	cv::Point3d normal;
	cv::Point3d point;
};

/**
 * Solves for the ray that starts at the camera center and passes through a
 * given pixel on the image plane. Output is expressed with respect to the
 * camera coordinate frame.
 *
 * \param intrinsic camera parameters; obtained by camera calibration
 * \param pt        point expressed in the image frame
 * \param dst       output ray stored as a column vector
 */
void GetPixelRay(cv::Mat mint, cv::Point2i pt, cv::Point3d &dst);

/**
 * Solves for the intersection of a line and a plane. This assumes that the
 * line passes through the origin.
 *
 * Assume the plane is parameterized by an arbitrary point on its surface,
 * \f$ p_p \f$, and a normal vector \f$ n \f$. Similarly, assume the line is
 * parameterized by an arbitrary point, \f$ p_l \f$, and a direction vector,
 * \f$ v \f$. The intersection point, \f$ p \f$, must satisfy
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
void GetRayPlaneInt(cv::Point3d ray, Plane plane, cv::Point3d &dst);

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
double GetDistSize(cv::Point2d pt0, double dist, cv::Mat mint, Plane plane);

/**
 * Extracts the intrinsic matrix from a ROS CameraInfo message into an OpenCV
 * matrix object. This should be replaced by an operation in CvBridge, if
 * equivalent functionality is added.
 *
 * \param msg  CameraInfo message
 * \param mint output intrinsic camera parameters
 */
void CameraInfoToMat(CameraInfoConstPtr const &msg, cv::Mat &mint);

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
 * \param ker   output parameter for the kernel
 * \param x     filter center
 * \param dim   width of the image to be filtered, in pixels
 * \param width filter width
 * \param fit   kernel is forced to have dim columns if false
 */
void BuildLineFilter(cv::Mat &ker, int x, int dim, int width, int border, bool fit = false);

/**
 * Transform a color 8-bit BGR image into a grayscale image where regions of
 * white are over-emphasized. Note that the input of this function is of type
 * CV_8U and the output it CV_64F (for future computations).
 * \param src color input image, 8-bit BGR
 * \param dst grayscale output image, 64-bit
 */
void LineColorTransform(cv::Mat src, cv::Mat &dst);

void NormalizeSaturation(cv::Mat src, cv::Mat &dst, cv::Size size);

void GuessGroundPlane(tf::TransformListener &tf,
                      std::string fr_gnd, std::string fr_cam, Plane &plane);

#endif

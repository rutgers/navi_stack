#ifndef LINE_DETECTION_NODE_HPP_
#define LINE_DETECTION_NODE_HPP_

#include <climits>
#include <cmath>
#include <string>
#include <vector>

#include <opencv/cv.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include "util.hpp"

namespace image_encodings = sensor_msgs::image_encodings;

using image_transport::CameraSubscriber;
using sensor_msgs::CameraInfoConstPtr;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;

class LineDetectionNode
{
public:
	/**
	 * Build a matched pulse-width filter using a line width and dead width
	 * specified in real-world coordinates. These are converted to camera
	 * coordinates using the specified intrinsic matrix and ground plane.
	 *
	 * \param nh        public node handle
	 * \param ground_id TF frame id for the ground plane
	 * \param debug     enable debugging topics
	 */
	LineDetectionNode(ros::NodeHandle nh, std::string ground_id, bool debug = false);

	void SetCutoffWidth(int  width);
	void SetDeadWidth(double width);
	void SetLineWidth(double width);
	void SetGroundPlane(Plane plane);
	void SetResolution(int width, int height);
	void SetThreshold(double threshold);

	void EstimateNormal(cv::Mat src, cv::Point2d &normal, cv::Point2i pt);

	/**
	 * Use a matched pulse-width filter to search for lines of the appropriate
	 * width in the image. This the output of this is an image with the same
	 * resolution as the input, where high values indicate a strong filter
	 * response and low values indicate a weak filter response.
	 *
	 * \param src     grayscale input image with type CV_64FC1
	 * \param dst_hor horizontal filter response of type CV_64FC1
	 * \param dst_ver vertical filter response of type CV_64FC1
	 */
	void MatchedFilter(cv::Mat src, cv::Mat &dst_hor, cv::Mat &dst_ver);

	/**
	 * Apply non-maximal supression to the output of the matched pulse-width
	 * filter to reduce the amount of data.
	 *
	 * \param src_hor horizontal filter response from MatchedFilter()
	 * \param src_ver vertical filter response from MatchedFilter()
	 * \param pts list of local maxima in the filter response
	 */
	void NonMaxSupr(cv::Mat src_hor, cv::Mat src_ver, std::list<cv::Point2i> &dst);

	/**
	 * Update cached information to match the current algorithmic parameters.
	 */
	void UpdateCache(void);

	void ImageCallback(ImageConstPtr const &msg_img, CameraInfoConstPtr const &msg_cam);

protected:
	struct Offset {
		int neg, pos;
	};

	cv::Point3d GetGroundPoint(cv::Point2d pt);
	double ProjectDistance(cv::Point2d pt, cv::Point3d offset);
	double ReprojectDistance(cv::Point2d pt, cv::Point2d offset);
	int GeneratePulseFilter(cv::Point3d dw, cv::Mat &kernel, std::vector<Offset> &offsets);
	void PulseFilter(cv::Mat src, cv::Mat &dst, cv::Mat kernel,
	                 std::vector<Offset> const &offsets, bool horizontal);
private:
	bool m_debug;
	bool m_valid;
	int m_rows;
	int m_cols;
	size_t m_num_prev;
	int    m_width_cutoff;
	double m_width_dead;
	double m_width_line;
	double m_threshold;
	Plane m_plane;
	std::string m_ground_id;

	int                 m_horizon_ver, m_horizon_hor;
	cv::Mat             m_kernel_ver,  m_kernel_hor;
	std::vector<Offset> m_offset_ver,  m_offset_hor;

	ros::NodeHandle                    m_nh;
	tf::TransformListener              m_tf;
	image_transport::ImageTransport    m_it;
	image_geometry::PinholeCameraModel m_model;

	CameraSubscriber           m_sub_cam;
	image_transport::Publisher m_pub_max;
	ros::Publisher             m_pub_pts;

	// Debug topics; only enabled if m_debug is true.
	image_transport::Publisher m_pub_distance;
	image_transport::Publisher m_pub_normal;
	image_transport::Publisher m_pub_filter_hor;
	image_transport::Publisher m_pub_filter_ver;
	image_transport::Publisher m_pub_ker_hor;
	image_transport::Publisher m_pub_ker_ver;
	ros::Publisher             m_pub_visual;
};

#endif

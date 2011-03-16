#ifndef LINE_DETECTION_NODE_HPP_
#define LINE_DETECTION_NODE_HPP_

#include <cmath>
#include <string>
#include <vector>
#include <opencv/cv.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include "util.hpp"

namespace image_encodings = sensor_msgs::image_encodings;

using image_transport::CameraSubscriber;
using sensor_msgs::CameraInfoConstPtr;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

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
	 */
	LineDetectionNode(ros::NodeHandle nh, std::string ground_id);

	void SetDeadWidth(double width);
	void SetLineWidth(double width);
	void SetGroundPlane(Plane plane);
	void SetResolution(int width, int height);
	void SetThreshold(double threshold);

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

private:
	bool m_valid;
	int m_rows;
	int m_cols;
	int m_horizon;
	double m_width_line;
	double m_width_dead;
	double m_threshold;
	Plane m_plane;
	cv::Mat m_mint;
	std::string m_ground_id;
	std::vector<double> m_cache_dead;
	std::vector<double> m_cache_line;

	ros::NodeHandle                 m_nh;
	tf::TransformListener           m_tf;
	image_transport::ImageTransport m_it;

	CameraSubscriber m_sub_cam;
	ros::Publisher   m_pub_pts;
};

#endif

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
	cv::Point3d forward;
};

/**
 * Transform a color 8-bit BGR image into a grayscale image where regions of
 * white are over-emphasized. Note that the input of this function is of type
 * CV_8U and the output it CV_64F (for future computations).
 * \param src color input image, 8-bit BGR
 * \param dst grayscale output image, 64-bit
 */
void LineColorTransform(cv::Mat src, cv::Mat &dst);

void GuessGroundPlane(tf::TransformListener &tf,
                      std::string fr_gnd, std::string fr_cam, Plane &plane);

#endif

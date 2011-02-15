#ifndef STEREO_IMAGE_PROC_PROCESSOR_H
#define STEREO_IMAGE_PROC_PROCESSOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <image_proc/processor.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace stereo_image_proc {

// Polymorphic wrapper for the various GPU-accelerated stereo correspondance
// algorithms.
class Matcher {
public:
  virtual void operator()(cv::Mat const &left,
                          cv::Mat const &right,
                          cv::Mat       &disparity) = 0;
  virtual void operator()(cv::Mat const &left,
                          cv::Mat const &right,
                          cv::Mat       &disparity,
                          cv::gpu::Stream const &stream) = 0;
};

template <typename T>
class MatcherWrapper : public Matcher {
public:
  MatcherWrapper(T &obj) : m_obj(obj) {}

  virtual void operator()(cv::Mat const &left,
                          cv::Mat const &right,
                          cv::Mat       &disparity) {

	m_obj.preset = cv::gpu::StereoBM_GPU::PREFILTER_XSOBEL;

	std::cout << "finding disparity" << std::endl;
    m_left  = left;
    m_right = right;
    m_obj(m_left, m_right, m_disparity);
    disparity = m_disparity;
	std::cout << "found disparity" << std::endl;
  }

  virtual void operator()(cv::Mat const &left,
                          cv::Mat const &right,
                          cv::Mat       &disparity,
                          cv::gpu::Stream const &stream) {
    m_left  = left;
    m_right = right;
    m_obj(m_left, m_right, m_disparity, stream);
    disparity = m_disparity;
  }

private:
  T &m_obj;
  cv::gpu::GpuMat m_left;
  cv::gpu::GpuMat m_right;
  cv::gpu::GpuMat m_disparity;
};

struct StereoImageSet
{
  image_proc::ImageSet left;
  image_proc::ImageSet right;
  stereo_msgs::DisparityImage disparity;
  sensor_msgs::PointCloud points;
  sensor_msgs::PointCloud2 points2;
};

class StereoProcessor
{
public:
  StereoProcessor(Matcher &matcher) : block_matcher_(matcher) {}

  enum {
    LEFT_MONO        = 1 << 0,
    LEFT_RECT        = 1 << 1,
    LEFT_COLOR       = 1 << 2,
    LEFT_RECT_COLOR  = 1 << 3,
    RIGHT_MONO       = 1 << 4,
    RIGHT_RECT       = 1 << 5,
    RIGHT_COLOR      = 1 << 6,
    RIGHT_RECT_COLOR = 1 << 7,
    DISPARITY        = 1 << 8,
    POINT_CLOUD      = 1 << 9,
    POINT_CLOUD2     = 1 << 10,

    LEFT_ALL = LEFT_MONO | LEFT_RECT | LEFT_COLOR | LEFT_RECT_COLOR,
    RIGHT_ALL = RIGHT_MONO | RIGHT_RECT | RIGHT_COLOR | RIGHT_RECT_COLOR,
    STEREO_ALL = DISPARITY | POINT_CLOUD | POINT_CLOUD2,
    ALL = LEFT_ALL | RIGHT_ALL | STEREO_ALL
  };

  // Do all the work!
  bool process(const sensor_msgs::ImageConstPtr& left_raw,
               const sensor_msgs::ImageConstPtr& right_raw,
               const image_geometry::StereoCameraModel& model,
               StereoImageSet& output, int flags) const;

  void processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                        const image_geometry::StereoCameraModel& model,
                        stereo_msgs::DisparityImage& disparity) const;

  void processPoints(const stereo_msgs::DisparityImage& disparity,
                     const cv::Mat& color, const std::string& encoding,
                     const image_geometry::StereoCameraModel& model,
                     sensor_msgs::PointCloud& points) const;
  void processPoints2(const stereo_msgs::DisparityImage& disparity,
                      const cv::Mat& color, const std::string& encoding,
                      const image_geometry::StereoCameraModel& model,
                      sensor_msgs::PointCloud2& points) const;

private:
  mutable image_proc::Processor mono_processor_;

  mutable Matcher &block_matcher_; // contains scratch buffers for block matching
  mutable cv::Mat_<int16_t> disparity16_; // scratch buffer for 16-bit signed disparity image
  // scratch buffers for speckle filtering
  mutable cv::Mat_<uint32_t> labels_;
  mutable cv::Mat_<uint32_t> wavefront_;
  mutable cv::Mat_<uint8_t> region_types_;
  // scratch buffer for dense point cloud
  mutable cv::Mat_<cv::Vec3f> dense_points_;
};

} //namespace stereo_image_proc

#endif

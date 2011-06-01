#ifndef MASK_NODE_HPP_
#define MASK_NODE_HPP_

#include <cmath>
#include <string>

#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_plane/Plane.h>

namespace navi_mask {

namespace mf = message_filters;

using sensor_msgs::CameraInfo;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud2;
using stereo_plane::Plane;
using pcl::PointXYZ;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef mf::sync_policies::ApproximateTime<LaserScan, CameraInfo, Plane> Policy;

class MaskNode {
public:
	void onInit(void);
	void transformLaserScanToPointCloud(std::string fr_fixed, LaserScan const &src, PointCloudXYZ &dst);
	void transformPlane(Plane const &src, Plane &dst, std::string frame_id);
	void ProjectPointCloud(PointCloudXYZ const &src, PointCloudXYZ &dst, Plane const &plane);
	void Callback(LaserScan::ConstPtr const &scan, CameraInfo::ConstPtr const &info, Plane::ConstPtr const &plane);

private:
	std::string m_fr_fixed;
	boost::shared_ptr<tf::TransformListener> m_tf;
	laser_geometry::LaserProjection m_projector;

	ros::Publisher m_pub_pts;

	mf::Subscriber<LaserScan>  *m_sub_laser;
	mf::Subscriber<CameraInfo> *m_sub_info;
	mf::Subscriber<Plane>      *m_sub_plane;
	mf::Synchronizer<Policy>   *m_sub;
};

};
#endif

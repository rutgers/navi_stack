#ifndef OD_NODE_HPP
#define OD_NODE_HPP

#include <nodelet/nodelet.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace od_node {

class StereoODNodelet : public nodelet::Nodelet {
public:
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

	virtual void onInit(void);
	float Distance(pcl::PointXYZ const &pt1, pcl::PointXYZ const &pt2);
	void FindObstacles(PointCloudXYZ const &src, PointCloudXYZ &dst);
	void Callback(PointCloudXYZ::ConstPtr const &msg_pts,
	              sensor_msgs::CameraInfo::ConstPtr const &msg_info);

private:
	int    m_pmin;
	double m_dmax;
	double m_hmin;
	double m_hmax;
	double m_theta;
	image_geometry::PinholeCameraModel m_model;

	ros::Publisher  m_pub_pts;
};
};
#endif

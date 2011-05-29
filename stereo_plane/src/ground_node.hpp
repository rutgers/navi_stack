#ifndef GROUND_NODE_HPP_
#define GROUND_NODE_HPP_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace stereo_plane {

class GroundNodelet {
public:
	GroundNodelet(void);
	ros::NodeHandle &getNodeHandle(void);
	ros::NodeHandle &getPrivateNodeHandle(void);
	virtual void onInit(void);

	void Callback(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const &msg);
	bool GetTFPlane(ros::Time stamp, std::string fr_fixed, std::string fr_ground, Plane &plane);
	bool GetSACPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const &pts, std::string fr_fixed, Plane &plane);
	void RenderPlane(Plane const &plane, double width, visualization_msgs::Marker &marker);

	double GetPlaneDistance(Plane const &pt1, Plane const &pt2);
	double GetPlaneAngle(Plane const &pt1, Plane const &pt2);

private:
	ros::NodeHandle nh, nh_priv;

	ros::Subscriber m_sub_pts;
	ros::Publisher  m_pub_plane;
	ros::Publisher  m_pub_viz;

	boost::shared_ptr<tf::TransformListener>    m_sub_tf;
	boost::shared_ptr<tf::TransformBroadcaster> m_pub_tf;

	Plane::Ptr m_prev;
	bool       m_valid_prev;

	int m_inliers_min;
	bool m_static;
	double m_cache_time;
	double m_range_max;
	double m_error_default;
	double m_error_inlier;
	double m_error_angle;
	std::string m_fr_fixed;
	std::string m_fr_default;
};
};
#endif

#ifndef LINE_TRACKER_NODE_HPP_
#define LINE_TRACKER_NODE_HPP_

#include <ros/ros.h>

typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;

class LineTrackerNode {
public:
	LineTrackerNode(ros::NodeHandle nh);
	void PointCloudCallback(PointNormalCloud::ConstPtr const &msg_pts);

private:
	ros::NodeHandle m_nh;
	ros::Subscriber m_sub_pts;
	std::vector<PointNormalCloud::ConstPtr> m_pts;
};

#endif

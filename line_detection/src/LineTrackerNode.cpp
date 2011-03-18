#include "LineTrackerNode.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

LineTrackerNode::LineTrackerNode(ros::NodeHandle nh)
	: m_nh(nh)
{
	m_sub_pts = nh.subscribe<PointNormalCloud>("line_points", 1, LineTrackerNode::PointCloudCallback, this);
}

void LineTrackerNode::PointCloudCallback(PointNormalCloud::ConstPtr const &msg_pts)
{
	// TODO: Use RANSAC to fit a piece-wise linear model.
	// TODO: Replace the piece-wise linear model with cubic splines.
	// TODO: Publish a peice-wise linear RViz Marker that approximates the spline fit.
}

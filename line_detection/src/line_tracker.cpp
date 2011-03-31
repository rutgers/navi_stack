#include "LineTrackerNode.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_tracker");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	int inliers;
	nh_priv.param<int>("inliers", inliers, 10);

	LineTrackerNode node(nh, inliers);

	ros::spin();
	return 0;
}

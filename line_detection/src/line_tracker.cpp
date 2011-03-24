#include "LineTrackerNode.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_tracker");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	LineTrackerNode node(nh);

	ros::spin();
	return 0;
}

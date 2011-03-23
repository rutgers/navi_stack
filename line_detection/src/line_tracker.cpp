#include "LineTrackerNode.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_detection");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// XXX: What happens if the node recieves a message before it is fully
	//      initialized by these function calls?
	LineTrackerNode node(nh, frame_id, debug);

	ros::spin();
	return 0;
}

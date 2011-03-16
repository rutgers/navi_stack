#include "LineDetectionNode.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_detection");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// Algorithmic parameters.
	double width_line, width_dead, threshold;
	std::string frame_id;

	nh_priv.param<double>("thickness", width_line, 0.0726);
	nh_priv.param<double>("border",    width_dead, 0.1452);
	nh_priv.param<double>("threshold", threshold,  40.0);
	nh_priv.param<std::string>("frame", frame_id, "base_footprint");

	// Image size for pre-computing some calculations.
	int width, height;

	nh_priv.param<int>("width",  width,  320);
	nh_priv.param<int>("height", height, 240);

	// XXX: What happens if the node recieves a message before it is fully
	//      initialized by these function calls?
	LineDetectionNode node(nh, frame_id);
	node.SetLineWidth(width_line);
	node.SetDeadWidth(width_dead);
	node.SetResolution(width, height);
	node.SetThreshold(threshold);

	ros::spin();
	return 0;
}

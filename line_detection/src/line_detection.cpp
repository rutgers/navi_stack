#include "LineDetectionNode.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_detection");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// Algorithmic parameters.
	bool debug, invert;
	double width_line, width_dead;
	double threshold;
	int width_cutoff;
	std::string frame_id;

	nh_priv.param<bool>("debug", debug, false);
	nh_priv.param<bool>("invert", invert, false);
	nh_priv.param<double>("thickness", width_line, 0.0726);
	nh_priv.param<double>("border", width_dead, 0.1452);
	nh_priv.param<double>("threshold", threshold, 30);
	nh_priv.param<int>("cutoff", width_cutoff, 2);
	nh_priv.param<std::string>("frame", frame_id, "base_footprint");

	// XXX: What happens if the node recieves a message before it is fully
	//      initialized by these function calls?
	LineDetectionNode node(nh, frame_id, debug);
	node.SetCutoffWidth(width_cutoff);
	node.SetLineWidth(width_line);
	node.SetDeadWidth(width_dead);
	node.SetThreshold(threshold);
	node.SetInvert(invert);

	ros::spin();
	return 0;
}

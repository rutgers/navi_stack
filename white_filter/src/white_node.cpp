#include <nodelet/loader.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "white_node");

	nodelet::Loader   manager(false);

	std::string name = ros::this_node::getName();
	std::vector<std::string> my_argv;
	nodelet::M_string        remappings;

	manager.load(name, "white_filter/white_nodelet", remappings, my_argv);

	ros::spin();
}

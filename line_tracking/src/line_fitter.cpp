#include "line_fitter.hpp"

#include <message_filters/Subscriber.h>
#include <message_filters/TimeSynchronizer.h>

PLUGINLIB_DECLARE_CLASS(line_tracking, fitter_node, tracker_node::FitterNodelet, nodelet::Nodelet)

namespace tracker_node {

void FitterNodelet::onInit(void)
{
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	mf::Subscriber<PointCloud3D> sub_left("line_left", 1);
	mf::Subscriber<PointCloud3D> sub_center("line_center", 1);
	mf::Subscriber<PointCloud3D> sub_right("line_right", 1);
	sub_sync = boost::make_shared<mf::TimeSynchronizer<PointCloud3D, PointCloud3D, PointCloud3D> >(sub_left, sub_center, sub_right, 1);
}

};

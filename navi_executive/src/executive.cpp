//#include <gps_common/conversions.h>
#include <vector>
#include <string>
#include <stdint.h>
#include <ros/ros.h>
#include <navi_executive/executive.h>
#include <navi_executive/AddWaypoint.h>
#include <navi_executive/Waypoint.h>

using namespace navi_executive;

bool addWaypointCallback(AddWaypoint::Request &request,
                         AddWaypoint::Response &response)
{
    ROS_INFO("Callback with %d waypoints.", static_cast<int>(request.waypoints.size()));
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "executive");
    ros::NodeHandle nh;

    ros::ServiceServer srv_add = nh.advertiseService("add_waypoint", &addWaypointCallback);

    ros::spin();
    return 0;
}

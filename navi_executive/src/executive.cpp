//#include <gps_common/conversions.h>
#include <vector>
#include <string>
#include <stdint.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <navi_executive/executive.h>
#include <navi_executive/AddWaypoint.h>
#include <navi_executive/Waypoint.h>

using move_base_msgs::MoveBaseAction;
using navi_executive::AddWaypoint;
using navi_executive::Waypoint;

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

    actionlib::SimpleActionClient<MoveBaseAction> act_goal("move_base/goal", true);
    ros::ServiceServer srv_add = nh.advertiseService("add_waypoint", &addWaypointCallback);

    act_goal.waitForServer();
    return 0;
}

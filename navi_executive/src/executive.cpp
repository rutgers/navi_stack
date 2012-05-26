//#include <gps_common/conversions.h>
#include <list>
#include <string>
#include <stdint.h>
#include <boost/lambda/lambda.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <nav_msgs/Odometry.h>
#include <navi_executive/executive.h>
#include <navi_executive/AddWaypoint.h>
#include <navi_executive/Waypoint.h>

using move_base_msgs::MoveBaseAction;
using move_base_msgs::MoveBaseGoal;
using navi_executive::AddWaypoint;
using navi_executive::Waypoint;

static std::list<std::list<Waypoint> > waypoints_;
static bool idle_ = true;

static void setGoal(Waypoint waypoint)
{
    MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.0; // ???
    goal.target_pose.pose.position.y = 0.0; // ???
    goal.target_pose.pose.orientation.w = 1.0;

    // TODO: Register a callback 
    act_goal_.sendGoal(goal);
}

static std::list<Waypoint>::iterator chooseGoal(std::list<Waypoint> &goals)
{
    // TODO: Choose a sane ordering for the goals.
    return goals.begin();
}

static bool addWaypointCallback(AddWaypoint::Request &request,
                                AddWaypoint::Response &response)
{
    // TODO: Convert the GPS coordinates into UTM.

    // Add a group that contains these waypoints to the end of the queue.
    std::list<Waypoint> const empty;
    std::list<Waypoint> &group = *waypoints_.insert(waypoints_.end(), empty);
    group.insert(group.begin(), request.waypoints.begin(), request.waypoints.end());

    // Choose a new goal if we were previously idle.
    if (idle_) {
        std::list<Waypoint>::iterator it = chooseGoal(group);
        Waypoint goal = *it;
        group.erase(it);
        setGoal(goal);
    }
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

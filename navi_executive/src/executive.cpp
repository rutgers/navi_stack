#include <list>
#include <string>
#include <stdint.h>
#include <boost/lambda/lambda.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <gps_common/conversions.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <nav_msgs/Odometry.h>
#include <navi_executive/executive.h>
#include <navi_executive/AddWaypoint.h>
#include <navi_executive/WaypointGPS.h>
#include <navi_executive/WaypointUTM.h>

using actionlib::SimpleClientGoalState;
using move_base_msgs::MoveBaseAction;
using move_base_msgs::MoveBaseResultConstPtr;
using move_base_msgs::MoveBaseGoal;
using navi_executive::AddWaypoint;
using navi_executive::WaypointGPS;
using navi_executive::WaypointUTM;

namespace navi_executive {

Executive::Executive(std::string add_topic, std::string goal_topic)
    : idle_(true)
    , act_goal_(goal_topic, true)
    , utm_frame_id_("/map")
{
    // These gymnastics are necessary to get around C++'s poor type inference.
    typedef boost::function<bool (AddWaypoint::Request &, AddWaypoint::Response &)> AddWaypointCallback;
    AddWaypointCallback callback = boost::bind(&Executive::addWaypointCallback, this, _1, _2);
    srv_add_ = nh_.advertiseService(add_topic, callback);

    act_goal_.waitForServer();
}

bool Executive::addWaypointCallback(AddWaypoint::Request &request,
                                    AddWaypoint::Response &response)
{
    std::list<WaypointUTM> const empty;
    std::list<WaypointUTM> &group = *waypoints_.insert(waypoints_.end(), empty);

    // Convert the GPS coordinates into UTM.
    for (size_t i = 0; i < group.size(); ++i) {
        WaypointUTM waypoint = convertGPStoUTM(request.waypoints[i]);
        group.push_back(waypoint);
    }

    // Choose a new goal if we were previously idle.
    if (idle_) {
        std::list<WaypointUTM>::iterator it = chooseGoal(group);
        WaypointUTM goal = *it;
        group.erase(it);
        setGoal(goal);
    }
    return true;
}

void Executive::goalDoneCallback(SimpleClientGoalState const &state,
                                 MoveBaseResultConstPtr const &result)
{
    
}

std::list<WaypointUTM>::iterator Executive::chooseGoal(std::list<WaypointUTM> &goals)
{
    // TODO: Choose a sane ordering for the goals.
    return goals.begin();
}

void Executive::setGoal(WaypointUTM waypoint)
{
    MoveBaseGoal goal;
    goal.target_pose.header.frame_id = utm_frame_id_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = waypoint.easting;
    goal.target_pose.pose.position.y = waypoint.northing;
    goal.target_pose.pose.orientation.w = 1.0;

    // TODO: Register a callback 
    typedef boost::function<void (SimpleClientGoalState const &,
                                  MoveBaseResultConstPtr const &)> GoalCallback;
    GoalCallback callback = boost::bind(&Executive::goalDoneCallback, this, _1, _2);
    act_goal_.sendGoal(goal, callback);
}

WaypointUTM Executive::convertGPStoUTM(WaypointGPS gps)
{
    WaypointUTM utm;
    gps_common::LLtoUTM(gps.lat, gps.lon, utm.northing, utm.easting, utm.zone);
    return utm;
}

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "executive");
    ros::NodeHandle nh;

    navi_executive::Executive executive("add_waypoint", "move_base/goal");
    return 0;
}

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

using move_base_msgs::MoveBaseAction;
using move_base_msgs::MoveBaseGoal;
using navi_executive::AddWaypoint;
using navi_executive::WaypointGPS;
using navi_executive::WaypointUTM;

static std::list<std::list<WaypointGPS> > waypoints_;
static bool idle_ = true;

namespace navi_executive {

Executive::Executive(std::string add_topic, std::string goal_topic)
    : idle_(true)
    , act_goal_(goal_topic, true)
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
    std::list<WaypointGPS> const empty;
    std::list<WaypointGPS> &group = *waypoints_.insert(waypoints_.end(), empty);

    // Convert the GPS coordinates into UTM.
    for (size_t i = 0; i < group.size(); ++i) {
        WaypointUTM waypoint = convertGPStoUTM(request.waypoints[i]);
        group.push_back(request.waypoints[i]);
    }

    // Choose a new goal if we were previously idle.
    if (idle_) {
        std::list<WaypointGPS>::iterator it = chooseGoal(group);
        WaypointGPS goal = *it;
        group.erase(it);
        setGoal(goal);
    }
    return true;
}

std::list<WaypointGPS>::iterator Executive::chooseGoal(std::list<WaypointGPS> &goals)
{
    // TODO: Choose a sane ordering for the goals.
    return goals.begin();
}

void Executive::setGoal(WaypointGPS waypoint)
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

#include <list>
#include <string>
#include <sstream>
#include <stdint.h>
#include <boost/lambda/lambda.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <gps_common/conversions.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <nav_msgs/Odometry.h>
#include <navi_executive/executive.h>
#include <navi_executive/AddWaypointGPS.h>
#include <navi_executive/AddWaypointUTM.h>
#include <navi_executive/WaypointGPS.h>
#include <navi_executive/WaypointUTM.h>

using actionlib::SimpleClientGoalState;
using move_base_msgs::MoveBaseAction;
using move_base_msgs::MoveBaseResultConstPtr;
using move_base_msgs::MoveBaseGoal;

namespace navi_executive {

Executive::Executive(std::string gps_topic, std::string utm_topic, std::string goal_topic)
    : idle_(true)
    , act_goal_(goal_topic, true)
    , utm_frame_id_("/map")
{
    // These gymnastics are necessary to get around C++'s poor type inference.
    typedef boost::function<bool (AddWaypointGPS::Request &, AddWaypointGPS::Response &)> AddWaypointGPSCallback;
    AddWaypointGPSCallback gps_callback = boost::bind(&Executive::addWaypointGPSCallback, this, _1, _2);
    srv_gps_ = nh_.advertiseService(gps_topic, gps_callback);

    typedef boost::function<bool (AddWaypointUTM::Request &, AddWaypointUTM::Response &)> AddWaypointUTMCallback;
    AddWaypointUTMCallback utm_callback = boost::bind(&Executive::addWaypointUTMCallback, this, _1, _2);
    srv_utm_ = nh_.advertiseService(utm_topic, utm_callback);
}

bool Executive::addWaypointUTMCallback(AddWaypointUTM::Request &request,
                                       AddWaypointUTM::Response &response)
{
    std::list<WaypointUTM> const empty;
    std::list<WaypointUTM> &group = *waypoints_.insert(waypoints_.end(), empty);

    // Convert the GPS coordinates into UTM.
    std::stringstream ss;
    for (size_t i = 0; i < request.waypoints.size(); ++i) {
        WaypointUTM waypoint_utm = request.waypoints[i];
        group.push_back(waypoint_utm);

        ss << " (" << waypoint_utm.northing << ", " << waypoint_utm.easting << ", " << waypoint_utm.zone << ")";
    }

    ROS_INFO("Queued Group:%s", ss.str().c_str());

    // Choose a new goal if we were previously idle.
    if (idle_) {
        advanceGoal();
    }
    return true;
}

bool Executive::addWaypointGPSCallback(AddWaypointGPS::Request &request,
                                       AddWaypointGPS::Response &response)
{
    std::list<WaypointUTM> const empty;
    std::list<WaypointUTM> &group = *waypoints_.insert(waypoints_.end(), empty);

    // Convert the GPS coordinates into UTM.
    std::stringstream ss;
    for (size_t i = 0; i < request.waypoints.size(); ++i) {
        WaypointGPS waypoint_gps = request.waypoints[i];
        WaypointUTM waypoint_utm = convertGPStoUTM(waypoint_gps);
        group.push_back(waypoint_utm);

        ss << " (" << waypoint_gps.lat << ", " << waypoint_gps.lon << ")";
    }

    ROS_INFO("Queued Group:%s", ss.str().c_str());

    // Choose a new goal if we were previously idle.
    if (idle_) {
        advanceGoal();
    }
    return true;
}

void Executive::goalDoneCallback(SimpleClientGoalState const &state,
                                 MoveBaseResultConstPtr const &result)
{
    if (state == SimpleClientGoalState::SUCCEEDED) {
        // TODO: Begin some recovery action.
        ROS_ERROR("Failed to reach goal.");
    }
    advanceGoal();
}

void Executive::advanceGoal(void)
{
    if (waypoints_.empty()) {
        idle_ = true;
        ROS_INFO("No waypoints remain.");
    } else {
        // TODO: Intelligently order the waypoints in the group instead of
        // arbitrarily selecting the first element.
        std::list<WaypointUTM> &group = waypoints_.front();
        WaypointUTM goal = group.front();
        setGoal(goal);
        group.pop_front();
        idle_ = false;

        ROS_INFO("Goal is UTM zone %s at (%f, %f).",
            goal.zone.c_str(), goal.northing, goal.easting);

        // Advance to the next waypoint group when the current one is empty.
        if (group.empty()) {
            waypoints_.pop_front();
        }
    }
}

void Executive::setGoal(WaypointUTM waypoint)
{
    MoveBaseGoal goal;
    goal.target_pose.header.frame_id = utm_frame_id_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = waypoint.easting;
    goal.target_pose.pose.position.y = waypoint.northing;
    goal.target_pose.pose.orientation.w = 1.0;

    // Register a callback to set the next goal.
    typedef boost::function<void (SimpleClientGoalState const &,
                                  MoveBaseResultConstPtr const &)> GoalCallback;
    GoalCallback callback = boost::bind(&Executive::goalDoneCallback, this, _1, _2);

    ROS_INFO("Waiting for server.");
    act_goal_.waitForServer();
    ROS_INFO("Sending goal.");
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

    navi_executive::Executive executive("add_waypoint", "move_base");
    ros::spin();
    return 0;
}

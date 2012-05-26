#ifndef EXECUTIVE_H_
#define EXECUTIVE_H_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <navi_executive/AddWaypoint.h>
#include <navi_executive/WaypointGPS.h>
#include <navi_executive/WaypointUTM.h>

namespace navi_executive {

class Executive {
public:
    Executive(std::string add_topic, std::string goal_topic);

private:
    bool idle_;
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> act_goal_;
    ros::ServiceServer srv_add_;

    bool addWaypointCallback(AddWaypoint::Request  &request,
                             AddWaypoint::Response &response);
    std::list<WaypointGPS>::iterator chooseGoal(std::list<WaypointGPS> &goals);
    void setGoal(WaypointGPS waypoint);
};

};


#endif

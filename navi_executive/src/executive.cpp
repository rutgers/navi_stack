//#include <gps_common/conversions.h>
#include <vector>
#include <string>
#include <stdint.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <navi_executive/executive.h>

using navi_executive::Waypoint;

namespace navi_executive {
};

static void checkParamType(XmlRpc::XmlRpcValue value, int type,
                           std::string const &error_msg)
{
    if (value.getType() != type) {
        ROS_FATAL("Error parsing waypoint list: %s", error_msg.c_str());
    }
}

static void parseWaypoint(XmlRpc::XmlRpcValue coords, Waypoint &waypoint)
{
    checkParamType(coords, XmlRpc::XmlRpcValue::TypeArray,
                   "Waypoint must be a list of two doubles.");
    if (coords.size() != 2) {
        ROS_FATAL("Waypoint must be a list of two doubles.");
    }

    checkParamType(coords[0], XmlRpc::XmlRpcValue::TypeDouble,
                   "Latitude must be a real number.");
    checkParamType(coords[1], XmlRpc::XmlRpcValue::TypeDouble,
                   "Longitude must be a real number.");
    waypoint.lat = static_cast<double>(coords[0]);
    waypoint.lon = static_cast<double>(coords[1]);
}

static void parseWaypointList(XmlRpc::XmlRpcValue waypoint_list,
                              std::vector<Waypoint> &waypoints)
{
    checkParamType(waypoint_list, XmlRpc::XmlRpcValue::TypeArray,
                   "Waypoint group must be a list of waypoints.");

    waypoints.resize(waypoint_list.size());
    for (int32_t i = 0; i < waypoint_list.size(); ++i) {
        parseWaypoint(waypoint_list[i], waypoints[i]);
    }
}

static void parseGroupList(XmlRpc::XmlRpcValue group_list,
                           std::vector<std::vector<Waypoint> > &groups)
{
    checkParamType(group_list, XmlRpc::XmlRpcValue::TypeArray,
                   "Expected a list of waypoint groups.");

    groups.resize(group_list.size());
    for (int32_t i = 0; i < group_list.size(); ++i) {
        parseWaypointList(group_list[i], groups[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "executive");
    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue waypoint_list;
    nh.getParam("waypoints", waypoint_list);

    std::vector<std::vector<Waypoint> > waypoints;
    parseGroupList(waypoint_list, waypoints);

    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "Group #" << i << ":\n";

        for (size_t j = 0; j < waypoints[i].size(); ++j) {
            std::cout << "- (" << waypoints[i][j].lat
                      << ", "  << waypoints[i][j].lon << ")\n";
        }
    }
    return 0;
}

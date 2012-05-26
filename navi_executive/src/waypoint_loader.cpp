#include <string>
#include <vector>
#include <sstream>
#include <stdint.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <navi_executive/executive.h>
#include <navi_executive/AddWaypoint.h>
#include <navi_executive/WaypointGPS.h>
#include <navi_executive/WaypointUTM.h>

using namespace navi_executive;

static void checkParamType(XmlRpc::XmlRpcValue value, int type,
                           std::string const &error_msg)
{
    if (value.getType() != type) {
        ROS_FATAL("Error parsing waypoint list: %s", error_msg.c_str());
    }
}

static void parseWaypoint(XmlRpc::XmlRpcValue coords, WaypointGPS &waypoint)
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
                              std::vector<WaypointGPS> &waypoints)
{
    checkParamType(waypoint_list, XmlRpc::XmlRpcValue::TypeArray,
                   "Waypoint group must be a list of waypoints.");

    waypoints.resize(waypoint_list.size());
    for (int32_t i = 0; i < waypoint_list.size(); ++i) {
        parseWaypoint(waypoint_list[i], waypoints[i]);
    }
}

static void parseGroupList(XmlRpc::XmlRpcValue group_list,
                           std::vector<std::vector<WaypointGPS> > &groups)
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
    ros::init(argc, argv, "waypoint_loader", ros::init_options::AnonymousName);
    ros::NodeHandle nh, nh_priv("~");

    // Load a list of waypoints from the parameter server.
    XmlRpc::XmlRpcValue waypoint_list;
    std::vector<std::vector<WaypointGPS> > waypoints;
    nh_priv.getParam("waypoints", waypoint_list);
    parseGroupList(waypoint_list, waypoints);

    if (waypoints.size() == 0) {
        ROS_FATAL("Waypoints parameter is empty.");
        return 0;
    }
    ROS_INFO("Found %d waypoints groups in the parameter.",
             static_cast<int>(waypoint_list.size()));

    // Iteratively add the waypoints to the executive.
    ros::ServiceClient srv = nh.serviceClient<AddWaypoint>("add_waypoint");
    srv.waitForExistence();

    for (size_t i = 0; i < waypoints.size(); ++i) {
        AddWaypoint::Request srv_request;
        AddWaypoint::Response srv_response;
        srv_request.waypoints = waypoints[i];
        srv.call(srv_request, srv_response);

        std::stringstream ss;
        for (size_t j = 0; j < waypoints[i].size(); ++j) {
            ss << " (" << waypoints[i][j].lat << ", " << waypoints[i][j].lon << ")";
        }

        ROS_INFO("Added Group #%d:%s", static_cast<int>(i + 1), ss.str().c_str());
    }
    return 0;
}

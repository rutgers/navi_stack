#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "util.hpp"

using geometry_msgs::PointStamped;
using geometry_msgs::Vector3Stamped;

void GuessGroundPlane(tf::TransformListener &tf,
                      std::string fr_gnd, std::string fr_cam, ros::Time stamp,
                      Plane &plane)
{
	// Assume the origin of the base_footprint frame is on the ground plane.
	PointStamped point_gnd, point_cam;
	point_gnd.header.stamp    = stamp;
	point_gnd.header.frame_id = fr_gnd;
	point_gnd.point.x = 0.0;
	point_gnd.point.y = 0.0;
	point_gnd.point.z = 0.0;

	// Assume the positive z-axis of the base_footprint frame is "up", implying
	// that it is normal to the ground plane.
	Vector3Stamped normal_gnd, normal_cam;
	normal_gnd.header.stamp    = stamp;
	normal_gnd.header.frame_id = fr_gnd;
	normal_gnd.vector.x = 0.0;
	normal_gnd.vector.y = 0.0;
	normal_gnd.vector.z = 1.0;

	// Forward direction of the robot.
	Vector3Stamped forward_gnd, forward_cam;
	forward_gnd.header.stamp    = stamp;
	forward_gnd.header.frame_id = fr_gnd;
	forward_gnd.vector.x = 1.0;
	forward_gnd.vector.y = 0.0;
	forward_gnd.vector.z = 0.0;

	// These may throw a tf::TransformException.
	tf.transformPoint(fr_cam, point_gnd, point_cam);
	tf.transformVector(fr_cam, normal_gnd, normal_cam);
	tf.transformVector(fr_cam, forward_gnd, forward_cam);

	// Convert from a StampedVector to the OpenCV data type.
	plane.point.x   = point_cam.point.x;
	plane.point.y   = point_cam.point.y;
	plane.point.z   = point_cam.point.z;
	plane.normal.x  = normal_cam.vector.x;
	plane.normal.y  = normal_cam.vector.y;
	plane.normal.z  = normal_cam.vector.z;
	plane.forward.x = forward_cam.vector.x;
	plane.forward.y = forward_cam.vector.y;
	plane.forward.z = forward_cam.vector.z;
}

#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>

#include "line_tracker.hpp"

PLUGINLIB_DECLARE_CLASS(line_tracking, tracker_node, tracker_node::TrackerNodelet, nodelet::Nodelet)

// FIXME: Shifted by half a grid cell.

namespace tracker_node {

void TrackerNodelet::onInit(void)
{
	ROS_ERROR("INIT line_tracker");
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	nh_priv.param<int>("grid_width",  m_grid_width,  2000);
	nh_priv.param<int>("grid_height", m_grid_height, 2000);
	nh_priv.param<double>("grid_size", m_grid_size, 0.10);
	nh_priv.param<double>("range_max", m_range_max, 10.0);
	nh_priv.param<std::string>("frame_robot", m_fr_robot, "/base_link");
	nh_priv.param<std::string>("frame_fixed", m_fr_fixed, "/map");

	m_tf = boost::make_shared<tf::TransformListener>(nh);

	m_grid.resize(m_grid_width * m_grid_height);
	for (int i = 0; i < m_grid_width * m_grid_height; ++i) {
		m_grid[i] = 0;
	}

	m_pub_ren = nh.advertise<GridCells>("line_occupancy", 1);
	m_sub_pts = nh.subscribe<PointCloud3D>("points", 10, &TrackerNodelet::AddPoints, this);
}

double TrackerNodelet::Distance(Point2D const &p1, Point2D const &p2) const
{
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

void TrackerNodelet::ProjectPoint(Point3D const &pt_3d, Point2D &pt_2d) const
{
	// FIXME: Mantain a constant distance along the ground plane.
	pt_2d.x = pt_3d.x;
	pt_2d.y = pt_3d.y;
}

void TrackerNodelet::GetRobotCenter(ros::Time stamp, Point2D &pt_2d) const
{
	geometry_msgs::PointStamped pt_robot;
	pt_robot.header.frame_id = m_fr_robot;
	pt_robot.header.stamp    = stamp;
	pt_robot.point.x = 0.0;
	pt_robot.point.y = 0.0;
	pt_robot.point.z = 0.0;

	geometry_msgs::PointStamped pt_fixed;
	m_tf->transformPoint(m_fr_fixed, pt_robot, pt_fixed);

	Point3D pt_3d;
	pt_3d.x = pt_fixed.point.x;
	pt_3d.y = pt_fixed.point.y;
	pt_3d.z = pt_fixed.point.z;
	ProjectPoint(pt_3d, pt_2d);
}

void TrackerNodelet::AddPoints(PointCloud3D::ConstPtr const &pts_3d)
{
	// Convert all the points to a frame that is fixed w.r.t. the world.
	PointCloud3D::Ptr pts_fixed = boost::make_shared<PointCloud3D>();
	try {
		pcl_ros::transformPointCloud(m_fr_fixed, *pts_3d, *pts_fixed, *m_tf);
	} catch (tf::TransformException const &e) {
		ROS_WARN("%s", e.what());
		return;
	}

	// Superimpose these points on the fixed cost map.
	for (size_t i = 0; i < pts_fixed->points.size(); ++i) {
		Point3D const &pt_3d = pts_fixed->points[i];

		if (!isnan(pt_3d.x) && !isnan(pt_3d.y) && !isnan(pt_3d.z)) {
			Point2D pt_2d;
			ProjectPoint(pt_3d, pt_2d);
			IncrementCell(pt_2d.x, pt_2d.y);
		}
	}

	// Render the map in RViz as a nav_msgs::GridCell message. Unfortunately
	// there is no non-binary equivant that can be easily visualized.
	GridCells::Ptr msg_ren = boost::make_shared<GridCells>();
	Point2D pt_robot;
	GetRobotCenter(pts_3d->header.stamp, pt_robot);

	for (int i = 0; i < m_grid_width * m_grid_height; ++i) {
		Point2D point = Index2Point(i);
		Value   value = m_grid[i];

		if (value > 0 && Distance(point, pt_robot) <= m_range_max) {
			geometry_msgs::Point cell;
			cell.x = point.x;
			cell.y = point.y;
			cell.z = 0.0;
			msg_ren->cells.push_back(cell);
		}
	}

	msg_ren->cell_width  = m_grid_size;
	msg_ren->cell_height = m_grid_size;
	msg_ren->header.stamp    = pts_3d->header.stamp;
	msg_ren->header.frame_id = m_fr_fixed;
	m_pub_ren.publish(msg_ren);
}

Value TrackerNodelet::GetCellValue(double x, double y) const
{
	int index = Point2Index(x, y);
	if (0 <= index && index <= m_grid_width * m_grid_height) {
		return m_grid[index];
	} else {
		return 0;
	}
}

void TrackerNodelet::IncrementCell(double x, double y)
{
	int index = Point2Index(x, y);

	if (0 <= index && index <= m_grid_width * m_grid_height) {
		ROS_ERROR("increment %d", index);
		if (m_grid[index] < 255) {
			++m_grid[index];
		}
	}
}

/*
 * PRIVATE
 */
void TrackerNodelet::Point2Grid(double x, double y, int &grid_x, int &grid_y) const
{
	grid_x = x / m_grid_size + (m_grid_width  / 2);
	grid_y = y / m_grid_size + (m_grid_height / 2);
}

int TrackerNodelet::Grid2Index(int grid_x, int grid_y) const
{
	return grid_y * m_grid_width + grid_x;
}

int TrackerNodelet::Point2Index(double x, double y) const
{
	int grid_x, grid_y;
	Point2Grid(x, y, grid_x, grid_y);
	return Grid2Index(grid_x, grid_y);
}

void TrackerNodelet::Index2Grid(int i, int &grid_x, int &grid_y) const
{
	grid_x = (i % m_grid_width) - (m_grid_width  / 2);
	grid_y = (i / m_grid_width) - (m_grid_height / 2);
}

Point2D TrackerNodelet::Grid2Point(int grid_x, int grid_y) const
{
	Point2D pt;
	pt.x = grid_x * m_grid_size;
	pt.y = grid_y * m_grid_size;
	return pt;
}

Point2D TrackerNodelet::Index2Point(int i) const
{
	int grid_x, grid_y;
	Index2Grid(i, grid_x, grid_y);
	return Grid2Point(grid_x, grid_y);
}

};

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
	ROS_ERROR("TRACKER init");

	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	nh_priv.param<int>("grid_width",  m_grid_width,  2000);
	nh_priv.param<int>("grid_height", m_grid_height, 2000);
	nh_priv.param<double>("grid_size", m_grid_size, 0.10);
	nh_priv.param<std::string>("frame_robot", m_fr_robot, "/base_link");
	nh_priv.param<std::string>("frame_fixed", m_fr_fixed, "/map");

	m_tf = boost::make_shared<tf::TransformListener>(nh);

	m_grid.resize(m_grid_width * m_grid_height);
	for (int i = 0; i < m_grid_width * m_grid_height; ++i) {
		m_grid[i] = 0;
	}

	m_pub_ren = nh.advertise<GridCells>("line_occupancy", 1);
	m_sub_pts = nh.subscribe<PointCloud3D>("line_points", 10, &TrackerNodelet::AddPoints, this);
}

void TrackerNodelet::ProjectPoint(Point3D const &pt_3d, Point2D &pt_2d) const
{
	// FIXME: Mantain a constant distance along the ground plane.
	pt_2d.x = pt_3d.x;
	pt_2d.y = pt_3d.y;
}

void TrackerNodelet::AddPoints(PointCloud3D::ConstPtr const &pts_3d)
{
	ROS_ERROR("TRACKER callback");

	// Convert all the points to a frame that is fixed w.r.t. the world.
	PointCloud3D::Ptr pts_fixed = boost::make_shared<PointCloud3D>();
	pcl_ros::transformPointCloud(m_fr_fixed, *pts_3d, *pts_fixed, *m_tf);

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

	for (int grid_y = 0; grid_y < m_grid_height; ++grid_y)
	for (int grid_x = 0; grid_x < m_grid_width;  ++grid_x) {
		Point2D center    = GetCellCenter(grid_x, grid_y);
		uint8_t intensity = m_grid[grid_y * m_grid_width + grid_x];

		if (intensity > 0) {
			geometry_msgs::Point cell;
			cell.x = center.x;
			cell.y = center.y;
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

Point2D TrackerNodelet::GetCellCenter(int grid_x, int grid_y) const
{
	Point2D pt;
	pt.x = (grid_x) * m_grid_size;
	pt.y = (grid_y) * m_grid_size;
	return pt;
}

uint8_t TrackerNodelet::GetCell(double x, double y) const
{
	int grid_x = (x) / m_grid_size;
	int grid_y = (y) / m_grid_size;
	int index  = grid_y * m_grid_width + grid_x;

	if (0 <= index && index <= m_grid_width * m_grid_height) {
		return m_grid[index];
	} else {
		return 0;
	}
}

void TrackerNodelet::IncrementCell(double x, double y)
{
	// FIXME: missing offset
	int grid_x = (x) / m_grid_size;
	int grid_y = (y) / m_grid_size;
	int index  = grid_y * m_grid_width + grid_x;

	//ROS_WARN("P(%f, %f) ---> [ %d, %d ]", x, y, grid_x, grid_y);
	if (0 <= index && index <= m_grid_width * m_grid_height) {
		//ROS_ERROR("INCREMENT [ %d, %d ]", grid_x, grid_y);
		if (m_grid[index] < 255) {
			++m_grid[index];
		}
	}
}


};

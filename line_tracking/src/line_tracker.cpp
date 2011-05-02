#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <opencv/cv.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

#include "line_tracker.hpp"

PLUGINLIB_DECLARE_CLASS(line_tracking, tracker_node, tracker_node::TrackerNodelet, nodelet::Nodelet)

// FIXME: Shifted by half a grid cell.

namespace tracker_node {

void TrackerNodelet::onInit(void)
{
	ros::NodeHandle &nh      = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	nh_priv.param<int>("grid_width",  m_grid_width,  2000);
	nh_priv.param<int>("grid_height", m_grid_height, 2000);
	nh_priv.param<double>("grid_size", m_grid_size, 0.10);
	nh_priv.param<double>("range_max", m_range_max, 10.0);
	nh_priv.param<std::string>("frame_robot", m_fr_robot, "/base_link");
	nh_priv.param<std::string>("frame_fixed", m_fr_fixed, "/map");

	m_tf = boost::make_shared<tf::TransformListener>(nh, ros::Duration(1.0));

	m_grid.resize(m_grid_width * m_grid_height);
	for (int i = 0; i < m_grid_width * m_grid_height; ++i) {
		m_grid[i] = 0;
	}

	m_pub_ren = nh.advertise<GridCells>("line_occupancy", 1);
	m_pub_viz = nh.advertise<Marker>("visualization_marker", 1);
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
	Point2D pt_robot;
	try {
		GetRobotCenter(pts_3d->header.stamp, pt_robot);
	} catch (tf::TransformException const &e) {
		ROS_WARN("%s", e.what());
		return;
	}

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

	// Fit a model to the accumulated data.
	std::vector<PointWeighted> pts;
	for (int i = 0; i < m_grid_width * m_grid_height; ++i) {
		Point2D point = Index2Point(i);
		Value   value = m_grid[i];

		if (value > 0 && Distance(point, pt_robot) <= m_range_max) {
			PointWeighted pt;
			pt.x = point.x;
			pt.y = point.y;
			pt.intensity = value;
			pts.push_back(pt);
		}
	}

#if 0
	LinearModel model;
	while ((int)pts.size() >= LinearModel::GetMinPoints()) {
		int inliers = FitModel(pts, 500, 0.30, model);

		// TODO: remove inliers and fit additional models
		break;
	}
#endif

	// Render the map as an image to get grayscale.
	int range_px = m_range_max / m_grid_size;
	cv::Mat img_ren(range_px, range_px, CV_8UC1, cv::Scalar(0));

	int x0, y0;
	Point2Grid(pt_robot.x, pt_robot.y, x0, y0);

	for (int dy = -range_px; dy <= +range_px; ++dy)
	for (int dx = -range_px; dx <= +range_px; ++dx) {
		int i = Grid2Index(x0 + dx, y0 + dy);

		if (0 <= i && i <= m_grid_width * m_grid_height) {
			img_ren.at<uint8_t>(dy + range_px, dx + range_px) = m_grid[i];
		} else {
			img_ren.at<uint8_t>(dy + range_px, dx + range_px) = 0;
		}
	}


	// Render the map in RViz as a nav_msgs::GridCell message. Unfortunately
	// there is no non-binary equivant that can be easily visualized.
	GridCells::Ptr msg_ren = boost::make_shared<GridCells>();

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

	// Render the line model in RViz.
#if 0
	Marker msg_viz = RenderModel(model);
	msg_viz.header.frame_id = m_fr_fixed;
	msg_viz.header.stamp    = pts_3d->header.stamp;
	m_pub_viz.publish(msg_viz);

	msg_ren->cell_width  = m_grid_size;
	msg_ren->cell_height = m_grid_size;
	msg_ren->header.stamp    = pts_3d->header.stamp;
	msg_ren->header.frame_id = m_fr_fixed;
	m_pub_ren.publish(msg_ren);
#endif
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
		if (m_grid[index] < 255) {
			++m_grid[index];
		}
	}
}

#define GetPointWeight(_x_) ((_x_).intensity)

template <class M>
int TrackerNodelet::FitModel(std::vector<PointWeighted> const &pts, int iterations, double threshold, M &model)
{
	ROS_ASSERT((int)pts.size() >= M::GetMinPoints());
	ROS_ASSERT(iterations >= 1);
	ROS_ASSERT(threshold  >= 0.0);

	M             best_model;
	std::set<int> best_inliers;
	int           best_support = 0;

	for (int it = 0; it < iterations; ++it) {
		M             maybe_model;
		std::set<int> maybe_inliers;
		int           maybe_support = 0;

		// Fit a model to randomly selected points.
		while ((int)maybe_inliers.size() < M::GetMinPoints()) {
			int i = rand() * pts.size() / RAND_MAX;
			maybe_inliers.insert(i);
			maybe_support += GetPointWeight(pts[i]);
		}

		std::vector<PointWeighted> seeds;
		std::set<int>::iterator seed_it;
		for (seed_it = maybe_inliers.begin(); seed_it != maybe_inliers.end(); ++seed_it) {
			seeds.push_back(pts[*seed_it]);
		}
		maybe_model.Fit(seeds);
		maybe_inliers.clear();

		// Add points that are sufficiently close to the model as inliers.
		for (int i = 0; i < (int)pts.size(); ++i) {
			if (maybe_model.Evaluate(pts[i]) <= threshold) {
				maybe_inliers.insert(i);
				maybe_support += GetPointWeight(pts[i]);
			}
		}

		// This model is better than the old best-known model.
		if (maybe_support > best_support) {
			best_model   = maybe_model;
			best_inliers = maybe_inliers;
			best_support = maybe_support;
		}
	}

	// TODO: Fit the model to all of the inliers.
	model = best_model;
	return best_support;
}

#define sgn(_x_) (((_x_) >= 0)?(+1):(-1))
Marker TrackerNodelet::RenderModel(LinearModel const &model) const
{
	Marker viz;
	viz.ns = "line_models";
	viz.id = 0;

	viz.type    = Marker::LINE_LIST;
	viz.scale.x = 0.1;
	viz.action  = Marker::ADD;

	viz.color.r = 0.0;
	viz.color.g = 1.0;
	viz.color.b = 0.0;
	viz.color.a = 1.0;

	viz.points.resize(2);

	double x1 = -100;
	double y1 = (model.c - model.a * x1) / model.b;

	double x2 = +100;
	double y2 = (model.c - model.a * x2) / model.b;

	viz.points[0].x = x1;
	viz.points[0].y = y1;
	viz.points[1].x = x2;
	viz.points[1].y = y2;
	return viz;
}

int LinearModel::GetMinPoints(void)
{
	return 2;
}

void LinearModel::Fit(std::vector<PointWeighted> const &pts)
{
	ROS_ASSERT(pts.size() == 2);
	a = pts[1].x - pts[0].x;
	b = pts[1].y - pts[0].y;
	c = a*(pts[0].x) + b*(pts[0].y);
}

double LinearModel::Evaluate(PointWeighted const &pt) const
{
	return (a*pt.x + b*pt.y + c) / sqrt(pow(a, 2) + pow(b, 2));
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

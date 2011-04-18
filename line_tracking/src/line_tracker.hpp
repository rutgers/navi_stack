#ifndef LINE_TRACKER_HPP_
#define LINE_TRACKER_HPP_

#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/GridCells.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

namespace tracker_node {

using nav_msgs::GridCells;

typedef pcl::PointXY   Point2D;
typedef pcl::PointXYZ  Point3D;
typedef pcl::PointCloud<Point2D>  PointCloud2D;
typedef pcl::PointCloud<Point3D>  PointCloud3D;

class TrackerNodelet : public nodelet::Nodelet {
public:
	virtual void onInit(void);
	void AddPoints(PointCloud3D::ConstPtr const &pts_3d);

	void ProjectPoint(Point3D const &pt_3d, Point2D &pt_2d) const;

	Point2D GetCellCenter(int grid_x, int grid_y) const;
	uint8_t GetCell(double x, double y) const;
	void IncrementCell(double x, double y);

private:
	int m_grid_width;
	int m_grid_height;
	double m_grid_size;
	std::string m_fr_fixed;
	std::string m_fr_robot;

	std::vector<uint8_t> m_grid;
	boost::shared_ptr<tf::TransformListener> m_tf;

	ros::Publisher  m_pub_ren;
	ros::Subscriber m_sub_pts;
};

};

#endif

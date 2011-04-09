#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

namespace mf = message_filters;

using sensor_msgs::CameraInfo;
using sensor_msgs::PointCloud2;

typedef pcl::PointCloud<pcl::PointXYZ>  PointCloudXYZ;

static double m_hmin;
static double m_hmax;
static double m_theta;
static float  float_nan = std::numeric_limits<float>::quiet_NaN();

static ros::Publisher  m_pub_pts;

float distance(pcl::PointXYZ const &pt1, pcl::PointXYZ const &pt2)
{
	return sqrt(pow(pt2.x - pt1.x, 2) + pow(pt2.y - pt1.y, 2) + pow(pt2.z - pt1.z, 2));
}

void FindObstacles(PointCloudXYZ const &src, PointCloudXYZ &dst,
                   double hmin, double hmax, double theta)
{
	float sin_theta = sin(m_theta);

	dst.points.resize(src.width * src.height);
	dst.width    = src.width;
	dst.height   = src.height;
	dst.is_dense = false;

	// Index the pointcloud using a k-d tree to make searching for points in
	// the truncated cones O(log n) intead of O(n).
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(src.makeShared());

	for (int y0 = 0; y0 < (int)src.height; ++y0)
	for (int x0 = 0; x0 < (int)src.width;  ++x0) {
		pcl::PointXYZ const &pt1    = src.points[y0 * src.width + x0];
		pcl::PointXYZ       &pt_dst = dst.points[y0 * src.width + x0];

		// Invalid points are marked with NaN.
		pt_dst.x = float_nan;
		pt_dst.y = float_nan;
		pt_dst.z = float_nan;
		if (isnan(pt1.x) || isnan(pt1.y) || isnan(pt1.z)) continue;

		// Use the kd-tree to restrict the search to the distance hmax.
		std::vector<int>   indices;
		std::vector<float> distances;
		int neighbors = tree.radiusSearch(pt1, m_hmax, indices, distances);

		// Two points are compatible iff their vertical offset is between hmin
		// and hmax and their angle to the plane is sufficiently large. if this
		// point is compatible with one or more other points, then it is an
		// obstacle.
		for (int i = 0; i < neighbors; ++i) {
			pcl::PointXYZ const &pt2 = src.points[indices[i]];

			if (indices[i] == (int)(y0 * src.width + x0)) continue;
			if (isnan(pt2.x) || isnan(pt2.y) || isnan(pt2.z)) continue;

			float height = fabs(pt2.y - pt1.y);
			float angle  = height / distance(pt1, pt2);
			bool valid_height = hmin <= height && height <= hmax;
			bool valid_angle  = angle >= sin_theta;

			if (valid_height && valid_angle) {
				pt_dst.x = pt1.x;
				pt_dst.y = pt1.y;
				pt_dst.z = pt1.z;
				break;
			}
		}
	}

	dst.width    = dst.points.size();
	dst.height   = 1;
	dst.is_dense = false;
}

void Callback(PointCloudXYZ::ConstPtr const &msg_pts, CameraInfo::ConstPtr const &msg_info)
{
	PointCloudXYZ obstacles;
	FindObstacles(*msg_pts, obstacles, m_hmin, m_hmax, m_theta);

	PointCloud2 msg_obstacles;
	pcl::toROSMsg(obstacles, msg_obstacles);

	msg_obstacles.header.stamp    = msg_pts->header.stamp;
	msg_obstacles.header.frame_id = msg_pts->header.frame_id;
	m_pub_pts.publish(msg_obstacles);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "od_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	nh_priv.param<double>("height_min", m_hmin,  0.3);
	nh_priv.param<double>("height_max", m_hmax,  2.0);
	nh_priv.param<double>("theta",      m_theta, M_PI / 4);

	mf::Subscriber<PointCloudXYZ> sub_pts(nh, "points", 1);
	mf::Subscriber<CameraInfo>    sub_info(nh, "camera_info", 1);
	mf::TimeSynchronizer<PointCloudXYZ, CameraInfo> sub_sync(sub_pts, sub_info, 10);
	sub_sync.registerCallback(&Callback);

	m_pub_pts = nh.advertise<PointCloudXYZ>("obstacle_points", 10);

	ros::spin();
	return 0;
}

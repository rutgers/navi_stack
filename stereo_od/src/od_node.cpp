#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ>  PointCloudXYZ;

static double m_hmin;
static double m_hmax;
static double m_theta;

static ros::Subscriber m_sub_pts;
static ros::Publisher  m_pub_pts;

float distance(pcl::PointXYZ const &pt1, pcl::PointXYZ const &pt2)
{
	return sqrt(pow(pt2.x - pt1.x, 2) + pow(pt2.y - pt1.y, 2) + pow(pt2.z - pt1.z, 2));
}

void FindObstacles(PointCloudXYZ const &src, PointCloudXYZ &dst,
                   double hmin, double hmax, double theta)
{
	dst.points.clear();

	// Index the pointcloud using a k-d tree to make searching for points in
	// the truncated cones O(log n) intead of O(n).
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(src.makeShared());

	int num_obs = 0;
	int num_all = src.width * src.height;

	for (size_t i = 0; i < src.points.size(); ++i) {
		pcl::PointXYZ const &pt1 = src.points[i];

		// Use the kd-tree to restrict the search to the distance hmax.
		std::vector<int>   indices;
		std::vector<float> distances;
		size_t neighbors = tree.radiusSearch(pt1, m_hmax, indices, distances);

		// Two points are compatible iff their vertical offset is between hmin
		// and hmax and their angle to the plane is sufficiently large. if this
		// point is compatible with one or more other points, then it is an
		// obstacle.
		for (size_t j = 0; j < neighbors; ++j) {
			if (indices[j] == (int)i) continue;

			pcl::PointXYZ const &pt2 = src.points[indices[j]];

			float angle  = abs(pt2.y - pt1.y) / distance(pt1, pt2);
			float height = abs(pt2.y - pt1.y);
			bool valid_height = hmin <= height && height >= hmax;
			bool valid_angle  = angle >= sin(theta);

			if (valid_height && valid_angle) {
				dst.points.push_back(pt1);
				++num_obs;
				break;
			}
		}
	}

	dst.width    = dst.points.size();
	dst.height   = 1;
	dst.is_dense = false;

	ROS_ERROR("Detected %d/%d points as obstacles.", num_obs, num_all);
}

void PointCloudCallback(PointCloudXYZ::ConstPtr const &pc_src)
{
	PointCloudXYZ pc_dst;

	ROS_ERROR("OD BEFORE");
	FindObstacles(*pc_src, pc_dst, m_hmin, m_hmax, m_theta);
	ROS_ERROR("OD AFTER");


#if 0
	pcl::toROSMsg(pc_dst, msg);
	//pc_dst.header.frame_id = pc_src.header.frame_id;
	//pc_dst.header.stamp    = pc_src.header.stamp;
#endif
	m_pub_pts.publish(pc_dst);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "od_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	nh_priv.param<double>("height_min", m_hmin,  0.100);
	nh_priv.param<double>("height_max", m_hmax,  1.000);
	nh_priv.param<double>("theta",      m_theta, M_PI / 2);

	m_sub_pts = nh.subscribe<PointCloudXYZ>("stereo_points", 1, &PointCloudCallback);
	m_pub_pts = nh.advertise<PointCloudXYZ>("obstacle_points", 10);

	ros::spin();
	return 0;
}

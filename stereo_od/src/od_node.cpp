#include <cmath>
#include <ros/ros.h>

#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/pending/disjoint_sets.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

namespace mf = message_filters;

using image_geometry::PinholeCameraModel;
using sensor_msgs::CameraInfo;
using sensor_msgs::PointCloud2;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

static int    m_pmin;
static double m_hmin;
static double m_hmax;
static double m_theta;
static PinholeCameraModel m_model;
static float  float_nan = std::numeric_limits<float>::quiet_NaN();

static ros::Publisher  m_pub_pts;

float dist(pcl::PointXYZ const &pt1, pcl::PointXYZ const &pt2)
{
	return sqrt(pow(pt2.x - pt1.x, 2) + pow(pt2.y - pt1.y, 2) + pow(pt2.z - pt1.z, 2));
}

void FindObstacles(PointCloudXYZ const &src, PointCloudXYZ &dst,
                   double hmin, double hmax, double flen, double theta)
{
	// Ranks is an associative data structure that stores the size of each
	// connected component. Parents stores the relationship between nodes for
	// use by path compression. Using raw arrays is more efficient than using
	// a std::map for these associative data structures.
	std::vector<int> rank(src.width * src.height);
	std::vector<int> parent(src.width * src.height);
	boost::disjoint_sets<int *, int *> djs(&rank[0], &parent[0]);

	// Begin with each pixel in its own connected component.
	boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> graph(src.width * src.height);
	boost::initialize_incremental_components(graph, djs);
	boost::incremental_components(graph, djs);

	float const sin_theta = sin(m_theta);
	float const tan_theta = tan(m_theta);

	dst.points.resize(src.width * src.height);
	dst.width    = src.width;
	dst.height   = src.height;
	dst.is_dense = false;

	// Initially mark all points as invalid (i.e. x = y = z = NaN).
	for (size_t i = 0; i < src.width * src.height; ++i) {
		dst.points[i].x = float_nan;
		dst.points[i].y = float_nan;
		dst.points[i].z = float_nan;
	}

	for (int y0 = src.height - 1; y0 >= 0; --y0)
	for (int x0 = src.width  - 1; x0 >= 0; --x0) {
		int const i = y0 * src.width + x0;
		pcl::PointXYZ const &pt1_src = src.points[i];
		if (isnan(pt1_src.x) || isnan(pt1_src.y) || isnan(pt1_src.z)) continue;

		// Project the cone above point P1 into the image as a trapezoid to
		// reduce the search space for points inside the cone. This reduces the
		// runtime of the algorithm from O(N^2) to O(K*N)
		int cone_height = m_hmax * pt1_src.z / flen;
		if (y0 - cone_height < 0) {
			cone_height = y0;
		}

		// Use the Manduchi OD2 algorithm. This exhaustively searches every
		// cone, examining each pair pair of pixels exactly once.
		for (int y = y0; y >= 0 && y >= y + cone_height; --y) {
			int cone_radius = (y0 - y) / tan_theta;
			int x_min = MAX(0,              x0 - cone_radius);
			int x_max = MIN((int)src.width, x0 + cone_radius + 1);

			for (int x = x_min; x < x_max; ++x) {
				int const j = y * src.width + x;
				pcl::PointXYZ const &pt2_src = src.points[j];
				if (isnan(pt2_src.x) || isnan(pt2_src.y) || isnan(pt2_src.z)) continue;

				// FIXME: height check may be redundant
				float height = fabs(pt2_src.y - pt1_src.y);
				float angle  = height / dist(pt1_src, pt2_src);
				bool valid_height = hmin <= height && height <= hmax;
				bool valid_angle  = angle >= sin_theta;

				if (valid_height && valid_angle) {
					djs.union_set(i, j);
				}
			}
		}
	}

	boost::component_index<int> components(parent.begin(), parent.end());

	BOOST_FOREACH(int component, components) {
		// FIXME: find the number of elements in the component
		int component_size = 0;
		BOOST_FOREACH(int index, components[component]) ++component_size;

		// Ignore components that are too small.
		if (component_size < m_pmin) continue;

		BOOST_FOREACH(int index, components[component]) {
			pcl::PointXYZ const &pt_src = src.points[index];
			pcl::PointXYZ       &pt_dst = dst.points[index];
			pt_dst.x = pt_src.x;
			pt_dst.y = pt_src.y;
			pt_dst.z = pt_src.z;
		}
	}

	dst.width  = src.width;
	dst.height = src.height;
	dst.is_dense = false;
}

void Callback(PointCloudXYZ::ConstPtr const &msg_pts, CameraInfo::ConstPtr const &msg_info)
{
	// Extract the camera's focal length from the CameraInfo message. Because
	// we are reprojecting a vertical distance, we can safely ignore fx().
	m_model.fromCameraInfo(msg_info);
	double flen = m_model.fy();

	PointCloudXYZ obstacles;
	FindObstacles(*msg_pts, obstacles, m_hmin, m_hmax, flen, m_theta);

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

	nh_priv.param<int>("height_min",    m_pmin,  25);
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

#include <cmath>
#include <ros/ros.h>

#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/pending/disjoint_sets.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <stereo_plane/Plane.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace mf = message_filters;

using image_geometry::PinholeCameraModel;
using sensor_msgs::CameraInfo;
using sensor_msgs::PointCloud2;
using stereo_plane::Plane;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

static bool m_simple;
static int m_pmin;
static double m_dmax;
static double m_hmin;
static double m_hmax;
static double m_pmax;
static double m_theta;
static PinholeCameraModel m_model;
static boost::shared_ptr<tf::TransformListener> m_tf;
static float  float_nan = std::numeric_limits<float>::quiet_NaN();

static ros::Publisher  m_pub_pts;

float dist(pcl::PointXYZ const &pt1, pcl::PointXYZ const &pt2)
{
	return sqrt(pow(pt2.x - pt1.x, 2) + pow(pt2.y - pt1.y, 2) + pow(pt2.z - pt1.z, 2));
}

float dist(Plane const &plane, pcl::PointXYZ const &pt)
{
	double a = plane.normal.x;
	double b = plane.normal.y;
	double c = plane.normal.z;
	double d = -(a * plane.point.x + b * plane.point.y + c * plane.point.z);
	return (a*pt.x + b*pt.y + c*pt.z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
}

void TransformPlane(Plane const &src, Plane &dst, std::string frame_id)
{
	geometry_msgs::PointStamped src_point;
	geometry_msgs::PointStamped dst_point;
	src_point.header = src.header;
	src_point.point  = src.point;

	geometry_msgs::Vector3Stamped src_normal;
	geometry_msgs::Vector3Stamped dst_normal;
	src_normal.header = src.header;
	src_normal.vector = src.normal;

	m_tf->transformPoint(frame_id, src_point, dst_point);
	m_tf->transformVector(frame_id, src_normal, dst_normal);

	dst.header = src.header;
	dst.point  = dst_point.point;
	dst.normal = dst_normal.vector;
	dst.type   = src.type;
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

	for (int y0 = src.height - 1; y0 >= 0; --y0)
	for (int x0 = src.width  - 1; x0 >= 0; --x0) {
		int const i = y0 * src.width + x0;
		pcl::PointXYZ const &pt1_src = src.points[i];

		if (pt1_src.z > m_dmax) continue;
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
				if (pt2_src.z > m_dmax) continue;

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
		// Ignore components that are too small.
		// FIXME: don't use a loop to check the size of each component
		int component_size = 0;
		BOOST_FOREACH(int index, components[component]) ++component_size;
		if (component_size < m_pmin) continue;

		BOOST_FOREACH(int index, components[component]) {
			pcl::PointXYZ const &pt_src = src.points[index];
			dst.points.push_back(pt_src);
		}
	}
}

void RemovePlane(PointCloudXYZ const &msg_src, PointCloudXYZ &msg_dst,
                 Plane const &plane, double height)
{
	// NOTE: msg_dst must be a copy of msg_src
	for (size_t i = 0; i < msg_dst.points.size(); ++i) {
		pcl::PointXYZ const &pt = msg_dst.points[i];

		if (dist(plane, pt) <= height) {
			msg_dst.points[i].x = float_nan;
			msg_dst.points[i].y = float_nan;
			msg_dst.points[i].z = float_nan;
		}
	}
}

void Callback(PointCloudXYZ::ConstPtr const &pts, CameraInfo::ConstPtr const &info,
              Plane::ConstPtr const &msg_plane)
{
	Plane plane;
	try {
		TransformPlane(*msg_plane, plane, pts->header.frame_id);
	} catch (tf::TransformException const &e) {
		ROS_WARN("%s", e.what());
		return;
	}

	// Make sure the normal vector is pointing "up".
	if (plane.normal.z < 0) {
		plane.normal.x *= -1;
		plane.normal.y *= -1;
		plane.normal.z *= -1;
	}

	// Extract the camera's focal length from the CameraInfo message. Because
	// we are reprojecting a vertical distance, we can safely ignore fx().
	m_model.fromCameraInfo(info);
	double flen = m_model.fy();

	// Remove points that are clearly on the ground plane to greatly speed up
	// the Manduchi OD algorithm.
	PointCloudXYZ::Ptr candidates = boost::make_shared<PointCloudXYZ>(*pts);
	RemovePlane(*pts, *candidates, plane, m_pmax);

	PointCloudXYZ::Ptr obstacles  = boost::make_shared<PointCloudXYZ>();

	if (!m_simple) {
		FindObstacles(*candidates, *obstacles, m_hmin, m_hmax, flen, m_theta);
	}

	obstacles->header.stamp    = pts->header.stamp;
	obstacles->header.frame_id = pts->header.frame_id;
	m_pub_pts.publish(obstacles);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "od_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	nh_priv.param<bool>("simple", m_simple, true);
	nh_priv.param<int>("points_min", m_pmin, 25);
	nh_priv.param<double>("distance_max", m_dmax,  5.0);
	nh_priv.param<double>("height_min",   m_hmin,  0.1);
	nh_priv.param<double>("height_max",   m_hmax,  2.0);
	nh_priv.param<double>("plane_max",    m_pmax,  0.3);
	nh_priv.param<double>("theta",        m_theta, M_PI / 4);

	mf::Subscriber<PointCloudXYZ> sub_pts(nh, "points", 1);
	mf::Subscriber<CameraInfo>    sub_info(nh, "camera_info", 1);
	mf::Subscriber<Plane>         sub_plane(nh, "ground_plane", 1);
	mf::TimeSynchronizer<PointCloudXYZ, CameraInfo, Plane> sub_sync(sub_pts, sub_info, sub_plane, 10);
	sub_sync.registerCallback(&Callback);

	m_tf = boost::make_shared<tf::TransformListener>(nh, ros::Duration(1.0));

	m_pub_pts = nh.advertise<PointCloud2>("obstacle_points", 10);

	ros::spin();
	return 0;
}

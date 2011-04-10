#include <cmath>
#include <ros/ros.h>
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

typedef pcl::PointCloud<pcl::PointXYZ>  PointCloudXYZ;

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
		pcl::PointXYZ const &pt1_src = src.points[y0 * src.width + x0];
		pcl::PointXYZ       &pt1_dst = dst.points[y0 * src.width + x0];

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
		bool is_obstacle = false;
		for (int y = y0; y >= 0 && y >= y + cone_height; --y) {
			int cone_radius = (y0 - y) / tan_theta;
			int x_min = MAX(0,              x0 - cone_radius);
			int x_max = MIN((int)src.width, x0 + cone_radius + 1);

			for (int x = x_min; x < x_max; ++x) {
				pcl::PointXYZ const &pt2_src = src.points[y * src.width + x];
				pcl::PointXYZ       &pt2_dst = dst.points[y * src.width + x];

				if (isnan(pt2_src.x) || isnan(pt2_src.y) || isnan(pt2_src.z)) continue;
				float height = fabs(pt2_src.y - pt1_src.y);
				float angle  = height / dist(pt1_src, pt2_src);
				bool valid_height = hmin <= height && height <= hmax;
				bool valid_angle  = angle >= sin_theta;

				if (valid_height && valid_angle) {
					is_obstacle = true;
					pt2_dst.x = pt2_src.x;
					pt2_dst.y = pt2_src.y;
					pt2_dst.z = pt2_src.z;
				}
			}
		}

		if (is_obstacle) {
			pt1_dst.x = pt1_src.x;
			pt1_dst.y = pt1_src.y;
			pt1_dst.z = pt1_src.z;
		}
	}

	dst.width    = dst.points.size();
	dst.height   = 1;
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

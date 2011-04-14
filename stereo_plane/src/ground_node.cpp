#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <stereo_plane/Plane.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

static int m_points_min;
static double m_freq;
static double m_range_max;
static double m_error_max;
static std::string m_fr_ground;
static std::string m_fr_fixed;
static std::string m_fr_default;

static ros::Publisher  m_pub_plane;
static ros::Publisher  m_pub_viz;
static ros::Subscriber m_sub_pts;

static tf::TransformListener    *m_sub_tf;
static tf::TransformBroadcaster *m_pub_tf;

void UpdateRender(std::string frame_id, stereo_plane::Plane plane,
                  pcl::ModelCoefficients coef)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp    = ros::Time::now();
	marker.type   = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.ns = "ground_plane";
	marker.id = 0;

	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.1;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	std::vector<geometry_msgs::Point> &pts = marker.points;
	std::vector<float>                &val = coef.values;
	pts.resize(5);
	pts[0].x = plane.point.x - 1.0;
	pts[0].y = plane.point.y - 1.0;
	pts[0].z = (-val[0] * pts[0].x + -val[1] * pts[0].y + -val[3]) / val[2];
	pts[1].x = plane.point.x + 1.0;
	pts[1].y = plane.point.y - 1.0;
	pts[1].z = (-val[0] * pts[1].x + -val[1] * pts[1].y + -val[3]) / val[2];
	pts[2].x = plane.point.x + 1.0;
	pts[2].y = plane.point.y + 1.0;
	pts[2].z = (-val[0] * pts[2].x + -val[1] * pts[2].y + -val[3]) / val[2];
	pts[3].x = plane.point.x - 1.0;
	pts[3].y = plane.point.y + 1.0;
	pts[3].z = (-val[0] * pts[3].x + -val[1] * pts[3].y + -val[3]) / val[2];
	pts[4].x = plane.point.x - 1.0;
	pts[4].y = plane.point.y - 1.0;
	pts[4].z = (-val[0] * pts[4].x + -val[1] * pts[4].y + -val[3]) / val[2];
	m_pub_viz.publish(marker);
}

void PointCloudCallback(PointCloudXYZ::ConstPtr const &pc_xyz)
{
	// Transform the point cloud into the base_link frame.
	PointCloudXYZ pc_frame;
	pcl_ros::transformPointCloud(m_fr_fixed, *pc_xyz, pc_frame, *m_sub_tf);

	// Prune points beyond the maximum range.
	pcl::PassThrough<pcl::PointXYZ> filter_pass;
	PointCloudXYZ pc_pass;
	filter_pass.setInputCloud(pc_frame.makeShared());
	filter_pass.setFilterFieldName("x");
	filter_pass.setFilterLimits(0.0, m_range_max);
	filter_pass.filter(pc_pass);

	// Fit a plane to the remaining points.
	pcl::ModelCoefficients::Ptr coef = boost::make_shared<pcl::ModelCoefficients>();
	pcl::PointIndices inliers;
	pcl::SACSegmentation<pcl::PointXYZ> filter_seg;
	filter_seg.setOptimizeCoefficients(true);
	filter_seg.setModelType(pcl::SACMODEL_PLANE);
	filter_seg.setMethodType(pcl::SAC_RANSAC);
	filter_seg.setDistanceThreshold(m_error_max);
	filter_seg.setInputCloud(pc_pass.makeShared());
	filter_seg.segment(inliers, *coef);

	// Only accept a model if it has a sufficient number of inliers. This helps
	// to reject ground planes that have been fit to obstacles or noise.
	stereo_plane::Plane plane;
	plane.header.frame_id = pc_xyz->header.frame_id;
	plane.header.stamp    = pc_xyz->header.stamp;

	if ((int)inliers.indices.size() >= m_points_min) {
		// Transform the origin of the fixed frame into the current frame.
		geometry_msgs::PointStamped pt_fix_ori;
		geometry_msgs::PointStamped pt_cam_ori;
		pt_fix_ori.header.frame_id = m_fr_fixed;
		pt_fix_ori.header.stamp    = pc_xyz->header.stamp;
		pt_fix_ori.point.x = 0.0;
		pt_fix_ori.point.y = 0.0;
		pt_fix_ori.point.z = 0.0;
		m_sub_tf->transformPoint(pc_xyz->header.frame_id, pt_fix_ori, pt_cam_ori);

		// Project the origin of the fixed frame onto the plane model.
		pcl::PointXYZ pt_ori;
		pt_ori.x = pt_cam_ori.point.x;
		pt_ori.y = pt_cam_ori.point.y;
		pt_ori.z = pt_cam_ori.point.z;

		PointCloudXYZ pc_origin;
		pc_origin.points.push_back(pt_ori);

		PointCloudXYZ pc_proj;
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(pc_origin.makeShared());
		proj.setModelCoefficients(coef);
		proj.filter(pc_proj);

		plane.point.x = pc_proj.points[0].x;
		plane.point.y = pc_proj.points[0].y;
		plane.point.z = pc_proj.points[0].z;
		plane.normal.x = coef->values[0];
		plane.normal.y = coef->values[1];
		plane.normal.z = coef->values[2];
	}
	// Default to the static transform specified specified by the robot's URDF.
	else {
		geometry_msgs::PointStamped pt_gnd;
		geometry_msgs::PointStamped pt_cam;
		pt_gnd.header.frame_id = m_fr_default;
		pt_gnd.header.stamp    = pc_xyz->header.stamp;
		pt_gnd.point.x = 0.0;
		pt_gnd.point.y = 0.0;
		pt_gnd.point.z = 0.0;
		m_sub_tf->transformPoint(pc_xyz->header.frame_id, pt_gnd, pt_cam);

		geometry_msgs::Vector3Stamped vec_gnd;
		geometry_msgs::Vector3Stamped vec_cam;
		vec_gnd.header.frame_id = m_fr_default;
		vec_gnd.header.stamp    = pc_xyz->header.stamp;
		vec_gnd.vector.x = 0.0;
		vec_gnd.vector.y = 0.0;
		vec_gnd.vector.z = 1.0;
		m_sub_tf->transformVector(pc_xyz->header.frame_id, vec_gnd, vec_cam);

		plane.point.x = pt_cam.point.x;
		plane.point.y = pt_cam.point.y;
		plane.point.z = pt_cam.point.z;
		plane.normal.x = vec_cam.vector.x;
		plane.normal.y = vec_cam.vector.y;
		plane.normal.z = vec_cam.vector.z;
	}
	m_pub_plane.publish(plane);

	UpdateRender(pc_xyz->header.frame_id, plane, *coef);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plane_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	m_sub_tf = new tf::TransformListener;
	m_pub_tf = new tf::TransformBroadcaster;
	m_pub_viz = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	m_pub_plane = nh.advertise<stereo_plane::Plane>("ground_plane", 10);

	nh_priv.param<int>("min_points", m_points_min, 10);
	nh_priv.param<double>("max_range", m_range_max, 3.00);
	nh_priv.param<double>("max_error", m_error_max, 0.05);
	nh_priv.param<double>("frequency", m_freq,      0.50);
	nh_priv.param<std::string>("frame_fixed",   m_fr_fixed,   "/base_link");
	nh_priv.param<std::string>("frame_ground",  m_fr_ground,  "/ground_link");
	nh_priv.param<std::string>("frame_default", m_fr_default, "/base_footprint");

	m_sub_pts = nh.subscribe<PointCloudXYZ>("stereo_points", 1, &PointCloudCallback);

	ros::spin();

	delete m_sub_tf;
	delete m_pub_tf;
	return 0;
}

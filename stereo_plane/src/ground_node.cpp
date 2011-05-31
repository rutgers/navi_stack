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
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <stereo_plane/Plane.h>

#include "ground_node.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

namespace stereo_plane {
// nodelet conversion
GroundNodelet::GroundNodelet(void)
	: nh_priv("~")
{}

ros::NodeHandle &GroundNodelet::getNodeHandle(void)
{
	return nh;
}

ros::NodeHandle &GroundNodelet::getPrivateNodeHandle(void)
{
	return nh_priv;
}
// nodelet conversion

void GroundNodelet::onInit(void)
{
	ros::NodeHandle nh      = getNodeHandle();
	ros::NodeHandle nh_priv = getPrivateNodeHandle();

	m_valid_prev = false;
	nh_priv.param<int>("inliers_min", m_inliers_min, 1000);
	nh_priv.param<int>("iterations",  m_iterations,   500);
	nh_priv.param<bool>("static", m_static, false);
	nh_priv.param<double>("probability",   m_prob,          0.99);
	nh_priv.param<double>("range_max",     m_range_max,     3.00);
	nh_priv.param<double>("error_default", m_error_default, 0.20);
	nh_priv.param<double>("error_inlier",  m_error_inlier,  0.20);
	nh_priv.param<double>("error_angle",   m_error_angle,   M_PI/6);
	nh_priv.param<double>("cache_time",    m_cache_time,    1.00);
	nh_priv.param<std::string>("frame_fixed",   m_fr_fixed,   "/base_link");
	nh_priv.param<std::string>("frame_default", m_fr_default, "/base_footprint");

	m_pub_tf = boost::make_shared<tf::TransformBroadcaster>();
	m_sub_tf = boost::make_shared<tf::TransformListener>(nh, ros::Duration(m_cache_time));

	m_pub_plane = nh.advertise<stereo_plane::Plane>("ground_plane", 10);
	m_pub_viz   = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	m_sub_pts   = nh.subscribe<PointCloudXYZ>("points", 1, &GroundNodelet::Callback, this);
}

void GroundNodelet::Callback(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const &msg_pts)
{
	ros::Time const &msg_stamp = msg_pts->header.stamp;

	// Wait for the TF buffer to catch up.
	m_sub_tf->waitForTransform(m_fr_fixed, m_fr_default, msg_stamp, ros::Duration(m_cache_time));
	m_sub_tf->waitForTransform(m_fr_fixed, msg_pts->header.frame_id, msg_stamp, ros::Duration(m_cache_time));

	// Get the default ground plane from TF (most likely a static transform in
	// in the robot's URDF). This is used as a sanity check.
	Plane::Ptr plane_def = boost::make_shared<Plane>();
	bool valid_def;
	try {
		valid_def = GetTFPlane(msg_stamp, m_fr_fixed, m_fr_default, *plane_def);
	} catch (tf::TransformException const &e) {
		ROS_WARN("%s", e.what());
		return;
	}

	// Detect the ground plane by using RANSAC to fit a plane to the stereo data.
	Plane::Ptr plane_fit = boost::make_shared<Plane>();
	bool valid_fit = GetSACPlane(msg_pts, m_fr_fixed, *plane_fit);

	// Perform a sanity check against our guess before accepting the fit.
	double distance  = GetPlaneDistance(*plane_def, *plane_fit);
	double angle     = GetPlaneAngle(*plane_def, *plane_fit);
	bool is_good_fit = distance <= m_error_default && angle <= m_error_angle;
	Plane::Ptr plane;

	if (valid_fit && valid_def && is_good_fit) {
		m_prev       = boost::make_shared<Plane>(*plane_fit);
		m_valid_prev = false;
		plane        = plane_fit;
		plane->type  = Plane::TYPE_FIT_NOW;
	} else if (m_valid_prev) {
		plane = m_prev;
		plane->type = Plane::TYPE_FIT_OLD;
	} else if (valid_def) {
		plane = plane_def;
		plane->type = Plane::TYPE_DEFAULT;
	} else {
		return;
	}
	plane->header.frame_id = m_fr_fixed;
	plane->header.stamp    = msg_stamp;

	visualization_msgs::Marker::Ptr viz = boost::make_shared<visualization_msgs::Marker>();
	RenderPlane(*plane, 0.5, *viz);

	m_pub_viz.publish(viz);
	m_pub_plane.publish(plane);
}


bool GroundNodelet::GetTFPlane(ros::Time stamp, std::string fr_fixed,
                               std::string fr_ground, Plane &plane)
{
	geometry_msgs::PointStamped pt_gnd;
	geometry_msgs::PointStamped pt_fix;
	pt_gnd.header.frame_id = fr_ground;
	pt_gnd.header.stamp    = stamp;
	pt_gnd.point.x = 0.0;
	pt_gnd.point.y = 0.0;
	pt_gnd.point.z = 0.0;

	geometry_msgs::Vector3Stamped vec_gnd;
	geometry_msgs::Vector3Stamped vec_fix;
	vec_gnd.header.frame_id = fr_ground;
	vec_gnd.header.stamp    = stamp;
	vec_gnd.vector.x = 0.0;
	vec_gnd.vector.y = 0.0;
	vec_gnd.vector.z = 1.0;

	try {
		m_sub_tf->transformPoint(fr_fixed, pt_gnd, pt_fix);
		m_sub_tf->transformVector(fr_fixed, vec_gnd, vec_fix);
	} catch (tf::TransformException const &e) {
		return false;
	}

	plane.point.x = pt_fix.point.x;
	plane.point.y = pt_fix.point.y;
	plane.point.z = pt_fix.point.z;
	plane.normal.x = vec_fix.vector.x;
	plane.normal.y = vec_fix.vector.y;
	plane.normal.z = vec_fix.vector.z;
	return true;
}

bool GroundNodelet::GetSACPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const &pts_const,
                                std::string fr_fixed, Plane &plane)
{
	// Transform the point cloud into the base_link frame.
	PointCloudXYZ::Ptr pc_temp  = boost::make_shared<PointCloudXYZ>(*pts_const);
	PointCloudXYZ::Ptr pc_frame = boost::make_shared<PointCloudXYZ>();
	try {
		pcl_ros::transformPointCloud(fr_fixed, *pc_temp, *pc_frame, *m_sub_tf);
	} catch (tf::TransformException const &e) {
		return false;
	}

	// Prune points beyond the maximum range.
	pcl::PassThrough<pcl::PointXYZ> filter_pass;
	PointCloudXYZ::Ptr pc_pass = boost::make_shared<PointCloudXYZ>();
	filter_pass.setInputCloud(pc_frame);
	filter_pass.setFilterFieldName("x");
	filter_pass.setFilterLimits(0.0, m_range_max);
	filter_pass.filter(*pc_pass);

	if (pc_pass->points.size() <= 0) return false;

	// Fit a plane to the remaining points.
	pcl::ModelCoefficients::Ptr coef = boost::make_shared<pcl::ModelCoefficients>();
	pcl::PointIndices inliers;
	pcl::SACSegmentation<pcl::PointXYZ> filter_seg;
	filter_seg.setOptimizeCoefficients(true);
	filter_seg.setModelType(pcl::SACMODEL_PLANE);
	filter_seg.setMethodType(pcl::SAC_RANSAC);
	filter_seg.setDistanceThreshold(m_error_inlier);
	filter_seg.setMaxIterations(m_iter);
	filter_seg.setProbability(m_prob);
	filter_seg.setInputCloud(pc_pass);
	filter_seg.segment(inliers, *coef);

	if ((int)inliers.indices.size() <= m_inliers_min) return false;

	// Project the origin of the fixed coordinate frame onto the plane. This is
	// the point on the plane that will be published.
	pcl::PointXYZ pt_ori(0.0, 0.0, 0.0);
	PointCloudXYZ::Ptr pc_origin = boost::make_shared<PointCloudXYZ>();
	pc_origin->points.push_back(pt_ori);

	PointCloudXYZ::Ptr pc_proj = boost::make_shared<PointCloudXYZ>();
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(pc_origin);
	proj.setModelCoefficients(coef);
	proj.filter(*pc_proj);

	plane.point.x = pc_proj->points[0].x;
	plane.point.y = pc_proj->points[0].y;
	plane.point.z = pc_proj->points[0].z;
	plane.normal.x = coef->values[0];
	plane.normal.y = coef->values[1];
	plane.normal.z = coef->values[2];
	return true;
}

void GroundNodelet::RenderPlane(Plane const &plane, double width,
                                visualization_msgs::Marker &marker)
{
	// Calculate the a*x + b*y + c*z + d = 0 form of the plane.
	std::vector<float> coef(4);
	coef[0] = plane.normal.x;
	coef[1] = plane.normal.y;
	coef[2] = plane.normal.z;
	coef[3] = -plane.normal.x * plane.point.x
	        + -plane.normal.y * plane.point.y
	        + -plane.normal.z * plane.point.z;

	marker.header.frame_id = plane.header.frame_id;
	marker.header.stamp    = plane.header.stamp;
	marker.type    = visualization_msgs::Marker::LINE_STRIP;
	marker.action  = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.05;
	marker.ns = "ground_plane";
	marker.id = 0;

	switch (plane.type) {
	case Plane::TYPE_DEFAULT:
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;
		break;

	case Plane::TYPE_FIT_OLD:
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;
		break;

	case Plane::TYPE_FIT_NOW:
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;
		break;

	default:
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.color.a = 1.0;
	}

	std::vector<geometry_msgs::Point> &pts = marker.points;
	pts.resize(5);
	pts[0].x = plane.point.x - width;
	pts[0].y = plane.point.y - width;
	pts[0].z = (-coef[0] * pts[0].x + -coef[1] * pts[0].y + -coef[3]) / coef[2];
	pts[1].x = plane.point.x - width;
	pts[1].y = plane.point.y + width;
	pts[1].z = (-coef[0] * pts[1].x + -coef[1] * pts[1].y + -coef[3]) / coef[2];
	pts[2].x = plane.point.x + width;
	pts[2].y = plane.point.y + width;
	pts[2].z = (-coef[0] * pts[2].x + -coef[1] * pts[2].y + -coef[3]) / coef[2];
	pts[3].x = plane.point.x + width;
	pts[3].y = plane.point.y - width;
	pts[3].z = (-coef[0] * pts[3].x + -coef[1] * pts[3].y + -coef[3]) / coef[2];
	pts[4].x = plane.point.x - width;
	pts[4].y = plane.point.y - width;
	pts[4].z = (-coef[0] * pts[4].x + -coef[1] * pts[4].y + -coef[3]) / coef[2];
}

#define DOT(_a_, _b_) ((_a_).x * (_b_).x + (_a_).y * (_b_).y + (_a_).z * (_b_).z)
#define NORM(_a_)     (sqrt(DOT(_a_, _a_)))

double GroundNodelet::GetPlaneDistance(Plane const &pt1, Plane const &pt2)
{
	geometry_msgs::Point pt;
	pt.x = pt2.point.x - pt1.point.x;
	pt.y = pt2.point.y - pt1.point.y;
	pt.z = pt2.point.z - pt1.point.z;
	return NORM(pt);
}

double GroundNodelet::GetPlaneAngle(Plane const &pt1, Plane const &pt2)
{
	return acos(DOT(pt1.normal, pt2.normal) / (NORM(pt1.normal) * NORM(pt2.normal)));
}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ground_node");
	stereo_plane::GroundNodelet node;
	node.onInit();
	ros::spin();
	return 0;
}

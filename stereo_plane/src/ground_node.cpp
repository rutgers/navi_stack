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
static double m_valid_distance;
static double m_valid_angle;
static std::string m_fr_ground;
static std::string m_fr_fixed;
static std::string m_fr_default;

static ros::Publisher  m_pub_plane;
static ros::Publisher  m_pub_viz;
static ros::Subscriber m_sub_pts;

static tf::TransformListener    *m_sub_tf;
static tf::TransformBroadcaster *m_pub_tf;

#define DOT(_a_, _b_) ((_a_).x * (_b_).x + (_a_).y * (_b_).y + (_a_).z * (_b_).z)
#define NORM(_a_)     (sqrt(DOT(_a_, _a_)))

void UpdateRender(std::string frame_id, ros::Time stamp, stereo_plane::Plane const &plane, bool fit)
{
	// Calculate the a*x + b*y + c*z + d = 0 form of the plane.
	std::vector<float> coef(4);
	coef[0] = plane.normal.x;
	coef[1] = plane.normal.y;
	coef[2] = plane.normal.z;
	coef[3] = -plane.normal.x * plane.point.x
	        + -plane.normal.y * plane.point.y
	        + -plane.normal.z * plane.point.z;

	visualization_msgs::Marker::Ptr marker = boost::make_shared<visualization_msgs::Marker>();
	marker->header.frame_id = frame_id;
	marker->header.stamp    = stamp;
	marker->type   = visualization_msgs::Marker::LINE_STRIP;
	marker->action = visualization_msgs::Marker::ADD;
	marker->ns = "ground_plane";
	marker->id = 0;

	marker->pose.position.x = 0.0;
	marker->pose.position.y = 0.0;
	marker->pose.position.z = 0.0;
	marker->pose.orientation.x = 0.0;
	marker->pose.orientation.y = 0.0;
	marker->pose.orientation.z = 0.0;
	marker->pose.orientation.w = 1.0;

	marker->scale.x = 0.1;

	if (fit) {
		marker->color.r = 0.0;
		marker->color.g = 1.0;
		marker->color.b = 0.0;
		marker->color.a = 1.0;
	} else {
		marker->color.r = 1.0;
		marker->color.g = 0.0;
		marker->color.b = 0.0;
		marker->color.a = 1.0;
	}

	std::vector<geometry_msgs::Point> &pts = marker->points;
	size_t i = 0;

	pts.resize(5);

	pts[i].x = plane.point.x - 1.0;
	pts[i].y = plane.point.y - 1.0;
	pts[i].z = (-coef[0] * pts[i].x + -coef[1] * pts[i].y + -coef[3]) / coef[2];
	++i;
	pts[i].x = plane.point.x - 1.0;
	pts[i].y = plane.point.y + 1.0;
	pts[i].z = (-coef[0] * pts[i].x + -coef[1] * pts[i].y + -coef[3]) / coef[2];
	++i;
	pts[i].x = plane.point.x + 1.0;
	pts[i].y = plane.point.y + 1.0;
	pts[i].z = (-coef[0] * pts[i].x + -coef[1] * pts[i].y + -coef[3]) / coef[2];
	++i;
	pts[i].x = plane.point.x + 1.0;
	pts[i].y = plane.point.y - 1.0;
	pts[i].z = (-coef[0] * pts[i].x + -coef[1] * pts[i].y + -coef[3]) / coef[2];
	++i;
	pts[i].x = plane.point.x - 1.0;
	pts[i].y = plane.point.y - 1.0;
	pts[i].z = (-coef[0] * pts[i].x + -coef[1] * pts[i].y + -coef[3]) / coef[2];

	m_pub_viz.publish(marker);
}

void CreatePlaneSAC(pcl::ModelCoefficients::ConstPtr const& coef,
                    stereo_plane::Plane &plane)
{
	pcl::PointXYZ pt_ori(0.0, 0.0, 0.0);
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

bool CreatePlaneTF(std::string frame_id, ros::Time stamp, stereo_plane::Plane &plane)
{
	// Defaults (just in case there is no tf transform).
	plane.point.x = 0.0;
	plane.point.y = 0.0;
	plane.point.z = 0.0;
	plane.normal.x = 0.0;
	plane.normal.y = 0.0;
	plane.normal.z = 1.0;

	geometry_msgs::PointStamped pt_gnd;
	geometry_msgs::PointStamped pt_fix;
	pt_gnd.header.frame_id = m_fr_default;
	pt_gnd.header.stamp    = stamp;
	pt_gnd.point.x = 0.0;
	pt_gnd.point.y = 0.0;
	pt_gnd.point.z = 0.0;

	geometry_msgs::Vector3Stamped vec_gnd;
	geometry_msgs::Vector3Stamped vec_fix;
	vec_gnd.header.frame_id = m_fr_default;
	vec_gnd.header.stamp    = stamp;
	vec_gnd.vector.x = 0.0;
	vec_gnd.vector.y = 0.0;
	vec_gnd.vector.z = 1.0;

	try {
		//m_sub_tf->waitForTransform(m_fr_default, pc_xyz->header.frame_id, pc_xyz->header.stamp, ros::Duration(1.0));
		m_sub_tf->transformPoint(frame_id, pt_gnd, pt_fix);
		m_sub_tf->transformVector(frame_id, vec_gnd, vec_fix);
	} catch (tf::TransformException const &e) {
		ROS_ERROR("unable to transform from \"%s\" to \"%s\": %s",
				  m_fr_default.c_str(), frame_id.c_str(), e.what());
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

void PointCloudCallback(PointCloudXYZ::ConstPtr const &pc_xyz)
{
	// Transform the point cloud into the base_link frame.
	PointCloudXYZ pc_frame;
	try {
		//m_sub_tf->waitForTransform(pc_xyz->header.frame_id, m_fr_fixed, pc_xyz->header.stamp, ros::Duration(1.0));
		pcl_ros::transformPointCloud(m_fr_fixed, *pc_xyz, pc_frame, *m_sub_tf);
	} catch (tf::TransformException const &e) {
		ROS_ERROR("unable to transform from \"%s\" to \"%s\": %s",
		          pc_xyz->header.frame_id.c_str(), m_fr_fixed.c_str(), e.what());
		return;
	}

	// Prune points beyond the maximum range.
	pcl::PassThrough<pcl::PointXYZ> filter_pass;
	PointCloudXYZ::Ptr pc_pass = boost::make_shared<PointCloudXYZ>();
	filter_pass.setInputCloud(pc_frame.makeShared());
	filter_pass.setFilterFieldName("x");
	filter_pass.setFilterLimits(0.0, m_range_max);
	filter_pass.filter(*pc_pass);

	// Fit a plane to the remaining points.
	pcl::ModelCoefficients::Ptr coef = boost::make_shared<pcl::ModelCoefficients>();
	pcl::PointIndices inliers;
	pcl::SACSegmentation<pcl::PointXYZ> filter_seg;
	filter_seg.setOptimizeCoefficients(true);
	filter_seg.setModelType(pcl::SACMODEL_PLANE);
	filter_seg.setMethodType(pcl::SAC_RANSAC);
	filter_seg.setDistanceThreshold(m_error_max);
	filter_seg.setInputCloud(pc_pass);
	filter_seg.segment(inliers, *coef);

	// Default plane specified by the user. This is used as a fallback when
	// the best-fit plane has too high error to be considered reliable.
	stereo_plane::Plane::Ptr plane = boost::make_shared<stereo_plane::Plane>();
	CreatePlaneTF(m_fr_fixed, pc_xyz->header.stamp, *plane);

	// Evaluate the fit against the default before accepting it. Only planes
	// within a user-defined angle and distance should be accepted.
	bool fit = false;

	if ((int)inliers.indices.size() >= m_points_min) {
		stereo_plane::Plane::Ptr plane_fit = boost::make_shared<stereo_plane::Plane>();
		CreatePlaneSAC(coef, *plane_fit);

		geometry_msgs::Point &pt1 = plane->point;
		geometry_msgs::Point &pt2 = plane_fit->point;
		double distance = sqrt(pow(pt2.x - pt1.x, 2) + pow(pt2.y - pt1.y, 2) + pow(pt2.z - pt1.z, 2));

		geometry_msgs::Vector3 &n1 = plane->normal;
		geometry_msgs::Vector3 &n2 = plane_fit->normal;
		double angle = acos(DOT(n1, n2) / (NORM(n1) * NORM(n2)));

		if (distance <= m_valid_distance && angle <= m_valid_angle) {
			plane = plane_fit;
			fit   = true;
		}
	}

	plane->header.frame_id = m_fr_fixed;
	plane->header.stamp    = pc_xyz->header.stamp;
	UpdateRender(m_fr_fixed, pc_xyz->header.stamp, *plane, fit);
	m_pub_plane.publish(plane);
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
	nh_priv.param<double>("valid_angle",    m_valid_angle,    M_PI / 6);
	nh_priv.param<double>("valid_distance", m_valid_distance, 0.20);
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

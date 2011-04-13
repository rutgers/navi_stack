#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

static int m_points_min;
static double m_freq;
static double m_range_max;
static double m_error_max;
static std::string m_fr_ground;
static std::string m_fr_fixed;
static std::string m_fr_default;

static tf::StampedTransform      m_transform;
static ros::Subscriber           m_sub_pts;
static tf::TransformListener    *m_sub_tf;
static tf::TransformBroadcaster *m_pub_tf;

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
	pcl::ModelCoefficients coef;
	pcl::PointIndices inliers;
	pcl::SACSegmentation<pcl::PointXYZ> filter_seg;
	filter_seg.setOptimizeCoefficients(true);
	filter_seg.setModelType(pcl::SACMODEL_PLANE);
	filter_seg.setMethodType(pcl::SAC_RANSAC);
	filter_seg.setDistanceThreshold(m_error_max);
	filter_seg.setInputCloud(pc_pass.makeShared());
	filter_seg.segment(inliers, coef);

	// Only accept a model if it has a sufficient number of inliers. This helps
	// to reject ground planes that have been fit to obstacles or noise.
	if ((int)inliers.indices.size() >= m_points_min) {
		pcl::PointXYZ const &pcl_point = pc_pass.points[inliers.indices[0]];
		tf::Vector3 tf_point(pcl_point.x, pcl_point.y, pcl_point.z);
		tf::Quaternion tf_angle(coef.values[0], coef.values[1], coef.values[2],
		                        coef.values[3]);

		tf::Transform transform;
		transform.setOrigin(tf_point);
		transform.setRotation(tf_angle);
		m_transform.setData(transform);
	}
	// Default to the static transform specified specified by the robot's URDF.
	else {
		m_sub_tf->lookupTransform(m_fr_fixed, m_fr_default, ros::Time(0), m_transform);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plane_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	m_sub_tf = new tf::TransformListener;
	m_pub_tf = new tf::TransformBroadcaster;

	nh_priv.param<int>("min_points", m_points_min, 10);
	nh_priv.param<double>("max_range", m_range_max, 3.00);
	nh_priv.param<double>("max_error", m_error_max, 0.05);
	nh_priv.param<double>("frequency", m_freq,      0.50);
	nh_priv.param<std::string>("frame_fixed",   m_fr_fixed,   "/base_link");
	nh_priv.param<std::string>("frame_ground",  m_fr_ground,  "/ground_link");
	nh_priv.param<std::string>("frame_default", m_fr_default, "/base_footprint");

	m_sub_pts = nh.subscribe<PointCloudXYZ>("stereo_points", 1, &PointCloudCallback);

	ros::Rate loop_rate(m_freq);

	for (;;) {
		m_transform.stamp_          = ros::Time::now();
		m_transform.frame_id_       = m_fr_fixed;
		m_transform.child_frame_id_ = m_fr_ground;
		m_pub_tf->sendTransform(m_transform);
		ros::spinOnce();
		loop_rate.sleep();
	}

	delete m_sub_tf;
	delete m_pub_tf;
	return 0;
}

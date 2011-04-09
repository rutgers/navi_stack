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


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

static std::string m_frame;
static std::string m_robot;
static double      m_range;
static int         m_number;

static ros::Subscriber           m_sub_pts;
static tf::TransformListener    *m_sub_tf;
static tf::TransformBroadcaster *m_pub_tf;

void PointCloudCallback(PointCloud::ConstPtr const &pc_xyz)
{
	// Transform the point cloud into the base_link frame.
	PointCloud pc_frame;
	pcl_ros::transformPointCloud(m_robot, *pc_xyz, pc_frame, *m_sub_tf);

	// Prune points beyond the maximum.
	pcl::PassThrough<pcl::PointXYZ> filter_pass;
	PointCloud pc_pass;
	filter_pass.setInputCloud(pc_frame.makeShared());
	filter_pass.setFilterFieldName("x");
	filter_pass.setFilterLimits(0.0, m_range);
	filter_pass.filter(pc_pass);

	// Fit a plane to the remaining points.
	pcl::ModelCoefficients coef;
	pcl::PointIndices inliers;
	pcl::SACSegmentation<pcl::PointXYZ> filter_seg;
	filter_seg.setOptimizeCoefficients(true);
	filter_seg.setModelType(pcl::SACMODEL_PLANE);
	filter_seg.setMethodType(pcl::SAC_RANSAC);
	filter_seg.setDistanceThreshold(0.01);
	filter_seg.setInputCloud(pc_pass.makeShared());
	filter_seg.segment(inliers, coef);

	if ((int)inliers.indices.size() >= m_number) {
		// Convert the model parameters to a properly oriented TF frame.
		pcl::PointXYZ const &pcl_point = pc_pass.points[inliers.indices[0]];
		tf::Vector3 tf_point(pcl_point.x, pcl_point.y, pcl_point.z);
		tf::Quaternion tf_angle(coef.values[0], coef.values[1], coef.values[2],
		                        coef.values[3]);

		tf::Transform tf_ground;
		tf_ground.setOrigin(tf_point);
		tf_ground.setRotation(tf_angle);
		m_pub_tf->sendTransform(tf::StampedTransform(tf_ground, pc_xyz->header.stamp, m_robot, m_frame));
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plane_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	m_sub_tf = new tf::TransformListener;
	m_pub_tf = new tf::TransformBroadcaster;

	nh_priv.param<double>("range", m_range, 3.0);
	nh_priv.param<int>("number", m_number, 10);
	nh_priv.param<std::string>("robot", m_robot, "/base_link");
	nh_priv.param<std::string>("frame", m_frame, "/ground_link");

	m_sub_pts = nh.subscribe<PointCloud>("stereo_points", 1, &PointCloudCallback);

	ros::spin();

	delete m_sub_tf;
	delete m_pub_tf;
}

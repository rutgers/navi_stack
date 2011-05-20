#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>

#include "line_mux.hpp"

PLUGINLIB_DECLARE_CLASS(line_tracking, mux_nodelet, line_tracking::MuxNodelet, nodelet::Nodelet)

namespace line_tracking {

void MuxNodelet::onInit(void)
{
	ros::NodeHandle nh      = getNodeHandle();
	ros::NodeHandle nh_priv = getPrivateNodeHandle();

	nh_priv.param<double>("cache_time", m_cache_time, 1.0);
	nh_priv.param<double>("sod_mean",   m_sod_mean,   10.0);
	nh_priv.param<double>("sod_stddev", m_sod_stddev, 0.5);
	nh_priv.param<std::string>("frame_fixed", m_fr_fixed, "/base_link");

	m_tf  = boost::make_shared<tf::TransformListener>(ros::Duration(m_cache_time));
	m_pub = nh.advertise<PointCloudXYZ>("line_points", 10);

	// FIXME: subscribers need to be wrapped in boost::shared_ptrs
	mf::Subscriber<PointCloudXYZ> *sub_pc1 = new mf::Subscriber<PointCloudXYZ>(nh, "line_points1", 1);
	mf::Subscriber<PointCloudXYZ> *sub_pc2 = new mf::Subscriber<PointCloudXYZ>(nh, "line_points2", 1);
	mf::Subscriber<PointCloudXYZ> *sub_pc3 = new mf::Subscriber<PointCloudXYZ>(nh, "line_points3", 1);
	Synchronizer *sub = new Synchronizer(SyncPolicy(20), *sub_pc1, *sub_pc2, *sub_pc3);
	sub->registerCallback(&MuxNodelet::Callback, this);

	// Can't use boost::make_shared() because of reference parameters.
	m_sub = boost::shared_ptr<Synchronizer>(sub);
}

void MuxNodelet::Callback(PointCloudXYZ::ConstPtr const &pc1,
                          PointCloudXYZ::ConstPtr const &pc2,
                          PointCloudXYZ::ConstPtr const &pc3)
{
	PointCloudXYZ::Ptr merged = boost::make_shared<PointCloudXYZ>();
	merged->header.stamp    = pc1->header.stamp;
	merged->header.frame_id = m_fr_fixed;

	// Transform the inputs into the fixed frame and build a single pointcloud.
	std::vector<PointCloudXYZ::ConstPtr> split;
	split.push_back(pc1);
	split.push_back(pc2);
	split.push_back(pc3);

	for (size_t i = 0; i < split.size(); ++i) {
		PointCloudXYZ::ConstPtr pc       = split[i];
		PointCloudXYZ::Ptr      pc_fixed = boost::make_shared<PointCloudXYZ>();

		try {
			m_tf->waitForTransform(m_fr_fixed, pc->header.frame_id, pc->header.stamp, ros::Duration(m_cache_time));
			pcl_ros::transformPointCloud(m_fr_fixed, *pc, *pc_fixed, *m_tf);
		} catch (tf::TransformException const &e) {
			NODELET_ERROR("%s", e.what());
			return;
		}
		merged->points.insert(merged->points.end(), pc_fixed->points.begin(), pc_fixed->points.end());
	}

	// Statistical Outlier Detection (SOD) filter.
	PointCloudXYZ::Ptr filtered = boost::make_shared<PointCloudXYZ>();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(merged);
	sor.setMeanK(m_sod_mean);
	sor.setStddevMulThresh(m_sod_stddev);
	sor.filter(*filtered);

	m_pub.publish(filtered);
}

};

#ifndef MUX_NODELET_HPP_
#define MUX_NODELET_HPP_

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

namespace line_tracking {

namespace mf = message_filters;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef mf::sync_policies::ApproximateTime<PointCloudXYZ, PointCloudXYZ, PointCloudXYZ> SyncPolicy;
typedef mf::Synchronizer<SyncPolicy> Synchronizer;

class MuxNodelet {
public:
	MuxNodelet(void);
	ros::NodeHandle &getNodeHandle(void);
	ros::NodeHandle &getPrivateNodeHandle(void);

	virtual void onInit(void);
	void Callback(PointCloudXYZ::ConstPtr const &pc1,
	              PointCloudXYZ::ConstPtr const &pc2,
	              PointCloudXYZ::ConstPtr const &pc3);

private:
	ros::NodeHandle nh, nh_priv;

	boost::shared_ptr<tf::TransformListener> m_tf;
	boost::shared_ptr<Synchronizer> m_sub;
	ros::Publisher m_pub;

	double m_cache_time;
	double m_sod_mean;
	double m_sod_stddev;
	std::string m_fr_fixed;
};

};

#endif

#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace mf = message_filters;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

using image_transport::CameraPublisher;
using image_transport::ImageTransport;
using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoConstPtr;
using sensor_msgs::Image;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::PointCloud2;

static CameraInfoManager *man_nl, *man_nr;
static CameraInfoManager *man_wl, *man_wr;

static CameraPublisher pub_nl, pub_nr;
static CameraPublisher pub_wl, pub_wr;
static ros::Publisher  pub_b;

static std::string info_nl, info_nr;
static std::string info_wl, info_wr;

static double min_n, max_n;
static double min_w, max_w;

void recieve(Image::ConstPtr const &msg_left, Image::ConstPtr const &msg_middle,
             Image::ConstPtr const &msg_right)
{
	ros::Time now = msg_left->header.stamp;

	CameraInfo info_nl = man_nl->getCameraInfo();
	CameraInfo info_nr = man_nr->getCameraInfo();
	CameraInfo info_wl = man_wl->getCameraInfo();
	CameraInfo info_wr = man_wr->getCameraInfo();

	// Synchronize the CameraInfo's with their respective images.
	info_nl.header.stamp = now;
	info_nr.header.stamp = now;
	info_wl.header.stamp = now;
	info_wr.header.stamp = now;
	info_nl.header.frame_id = msg_left->header.frame_id;
	info_nr.header.frame_id = msg_middle->header.frame_id;
	info_wl.header.frame_id = msg_left->header.frame_id;
	info_wr.header.frame_id = msg_right->header.frame_id;

	// Duplicate the left camera with two sets of calibration parameters.
	pub_nl.publish(*msg_left,   info_nl);
	pub_nr.publish(*msg_middle, info_nr);
	pub_wl.publish(*msg_left,   info_wl);
	pub_wr.publish(*msg_right,  info_wr);
}

void merge(PointCloud2::ConstPtr const &pts2_narrow, PointCloud2::ConstPtr const &pts2_wide)
{
	PointCloudXYZ::Ptr pts_narrow = boost::make_shared<PointCloudXYZ>();
	PointCloudXYZ::Ptr pts_wide   = boost::make_shared<PointCloudXYZ>();
	pcl::fromROSMsg(*pts2_narrow, *pts_narrow);
	pcl::fromROSMsg(*pts2_wide, *pts_wide);

	PointCloudXYZ::Ptr pts = boost::make_shared<PointCloudXYZ>();
	pts->width  = pts_narrow->width;
	pts->height = pts_narrow->height;
	pts->points.resize(pts->width * pts->height);

	pcl::PointXYZ pt_nan;
	pt_nan.x = std::numeric_limits<double>::quiet_NaN();
	pt_nan.y = std::numeric_limits<double>::quiet_NaN();
	pt_nan.z = std::numeric_limits<double>::quiet_NaN();

	// Use wide baseline where available; otherwise fall back on narrow.
	for (size_t i = 0; i < pts_narrow->width * pts_narrow->height; ++i) {
		pcl::PointXYZ &pt_n = pts_narrow->points[i];
		pcl::PointXYZ &pt_w = pts_wide->points[i];
		pcl::PointXYZ &pt_b = pts->points[i];
		bool valid_n = isnan(pt_n.x) || isnan(pt_n.y) || isnan(pt_n.z);
		bool valid_w = isnan(pt_w.x) || isnan(pt_w.y) || isnan(pt_w.z);

		if (valid_n && min_w <= pt_w.z && pt_w.z <= max_w) {
			pt_b = pt_w;
		} else if (valid_w && min_n <= pt_n.z && pt_n.z <= max_n) {
			pt_b = pt_n;
		} else {
			pt_b = pt_nan;
		}
	}

	pts->header.stamp    = pts_narrow->header.stamp;
	pts->header.frame_id = pts_narrow->header.frame_id;
	pub_b.publish(pts);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "switcher_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::NodeHandle nh_nl("narrow/left"), nh_nr("narrow/right");
	ros::NodeHandle nh_wl("wide/left"),   nh_wr("wide/right");
	ImageTransport it(nh);

	// Use completely separate calibration parameters for the narrow and wide
	// camera pairs even though one of the cameras is shared. This is
	// necessary to preserve the integrity of the recitification.
	nh_priv.param<std::string>("info_nl", info_nl, "");
	nh_priv.param<std::string>("info_nr", info_nr, "");
	nh_priv.param<std::string>("info_wl", info_wl, "");
	nh_priv.param<std::string>("info_wr", info_wr, "");

	man_nl = new CameraInfoManager(nh_nl, "", info_nl);
	man_nr = new CameraInfoManager(nh_nr, "", info_nr);
	man_wl = new CameraInfoManager(nh_wl, "", info_wl);
	man_wr = new CameraInfoManager(nh_wr, "", info_wr);

	// Cap the output FPS at a fixed rate.
	nh_priv.param<double>("narrow_min", min_n, 0.0);
	nh_priv.param<double>("narrow_max", max_n, INFINITY);
	nh_priv.param<double>("wide_min",   min_w, 0.0);
	nh_priv.param<double>("wide_max",   max_w, INFINITY);

	// Republish the source images with new camera_info messages.
	std::string topic_n  = nh.resolveName("narrow");
	std::string topic_nl = nh.resolveName(topic_n + "/left");
	std::string topic_nr = nh.resolveName(topic_n + "/right");

	std::string topic_w  = nh.resolveName("wide");
	std::string topic_wl = nh.resolveName(topic_w + "/left");
	std::string topic_wr = nh.resolveName(topic_w + "/right");

	pub_nl = it.advertiseCamera(topic_nl + "/image", 1);
	pub_nr = it.advertiseCamera(topic_nr + "/image", 1);
	pub_wl = it.advertiseCamera(topic_wl + "/image", 1);
	pub_wr = it.advertiseCamera(topic_wr + "/image", 1);
	pub_b  = nh.advertise<PointCloudXYZ>("points", 1);

	// Always subscribe to synchronized (left, middle, right) image triplets;
	// we will handle the multiplexing entirely in the callback.
	std::string topic_l = nh.resolveName("left");
	std::string topic_m = nh.resolveName("middle");
	std::string topic_r = nh.resolveName("right");
	mf::Subscriber<Image> sub_l(nh, topic_l + "/image", 1);
	mf::Subscriber<Image> sub_m(nh, topic_m + "/image", 1);
	mf::Subscriber<Image> sub_r(nh, topic_r + "/image", 1);
	mf::TimeSynchronizer<Image, Image, Image> sync_img(sub_l, sub_m, sub_r, 10);
	sync_img.registerCallback(boost::bind(&recieve, _1, _2, _3));

	// Merge the stereo output.
	mf::Subscriber<PointCloud2> sub_n(nh, topic_n + "/points2", 1);
	mf::Subscriber<PointCloud2> sub_w(nh, topic_w + "/points2", 1);
	mf::TimeSynchronizer<PointCloud2, PointCloud2> sync_pts(sub_n, sub_w, 10);
	sync_pts.registerCallback(boost::bind(&merge, _1, _2));

	ros::spin();

	delete man_nl;
	delete man_nr;
	delete man_wl;
	delete man_wr;
}

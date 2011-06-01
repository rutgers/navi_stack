#include "mask_node.hpp"

namespace navi_mask {

void MaskNode::onInit(void)
{
	ros::NodeHandle nh, nh_priv("~");

	nh_priv.param<std::string>("fr_fixed", m_fr_fixed, "/base_link");

	m_tf = boost::make_shared<tf::TransformListener>(nh, ros::Duration(1.0));

	m_pub_pts   = nh.advertise<PointCloudXYZ>("laser_ground", 1);
	m_sub_laser = new mf::Subscriber<LaserScan>(nh, "laser_scan", 1);
	m_sub_info  = new mf::Subscriber<CameraInfo>(nh, "camera_info", 1);
	m_sub_plane = new mf::Subscriber<Plane>(nh, "ground_plane", 1);
	m_sub = new mf::Synchronizer<Policy>(Policy(30), *m_sub_laser, *m_sub_info, *m_sub_plane);
	m_sub->registerCallback(&MaskNode::Callback, this);
}

void MaskNode::ProjectPointCloud(PointCloudXYZ const &src,
                                 PointCloudXYZ       &dst,
                                 Plane         const &plane)
{
	dst.points.resize(src.points.size());

	// Force the normal to be unit length to simplify the projection.
	double norm = sqrt(pow(plane.normal.x, 2)
	                 + pow(plane.normal.y, 2)
	                 + pow(plane.normal.z, 2));
	double a = plane.normal.x / norm;
	double b = plane.normal.y / norm;
	double c = plane.normal.z / norm;
	double d = -(a * plane.point.x + b * plane.point.y + c * plane.point.z);

	// Point-wise projection
	for (size_t i = 0; i < src.points.size(); ++i) {
		PointXYZ const &src_pt = src.points[i];
		PointXYZ       &dst_pt = dst.points[i];

		double t = a * src_pt.x + b * src_pt.y + c * src_pt.z + d;
		dst_pt.x = src_pt.x - a * t;
		dst_pt.y = src_pt.y - b * t;
		dst_pt.z = src_pt.z - c * t;
	}
}

void MaskNode::transformLaserScanToPointCloud(std::string      fr_fixed,
                                              LaserScan const &src,
                                              PointCloudXYZ   &dst)
{
	PointCloud2 dst_pc2;
	m_projector.transformLaserScanToPointCloud(fr_fixed, src, dst_pc2, *m_tf);
	pcl::fromROSMsg(dst_pc2, dst);
}

void MaskNode::transformPlane(Plane const &src, Plane &dst, std::string frame_id)
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

void MaskNode::Callback(LaserScan::ConstPtr  const &scan,
                        CameraInfo::ConstPtr const &info,
                        Plane::ConstPtr      const &plane)
{
	ros::Time   const &time_camera = info->header.stamp;
	std::string const &fr_camera   = info->header.frame_id;

	// Convert the laser scan into a point cloud to simplify projection.
	PointCloudXYZ::Ptr scan_fixed  = boost::make_shared<PointCloudXYZ>();
	PointCloudXYZ::Ptr scan_camera = boost::make_shared<PointCloudXYZ>();
	Plane plane_camera;

	try {
		transformPlane(*plane, plane_camera, fr_camera);
		transformLaserScanToPointCloud(m_fr_fixed, *scan, *scan_fixed);
		pcl_ros::transformPointCloud(fr_camera, time_camera, *scan_fixed, m_fr_fixed, *scan_camera, *m_tf);
	} catch (tf::TransformException const &e) {
		ROS_WARN("%s", e.what());
		return;
	}

	// Project the laser scan on to the ground plane.
	PointCloudXYZ::Ptr scan_ground = boost::make_shared<PointCloudXYZ>();
	ProjectPointCloud(*scan_camera, *scan_ground, plane_camera);

	// TODO: create a mask image

	scan_ground->header.stamp    = info->header.stamp;
	scan_ground->header.frame_id = fr_camera;
	m_pub_pts.publish(scan_ground);
}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mask_node");
	navi_mask::MaskNode node;
	node.onInit();
	ros::spin();
	return 0;
}

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

struct Plane {
	cv::Point3d point;
	cv::Vec3d   normal;
};

static double m_minz;
static double m_maxz;
static double m_theta;
static std::string m_plane;

static ros::Subscriber           m_sub_pts;
static tf::TransformListener    *m_sub_tf;
static tf::TransformBroadcaster *m_pub_tf;

cv::Point3d StereoObstacleDetection::GetGroundPoint(cv::Point2d pt)
{
	cv::Point3d ray     = m_model.projectPixelTo3dRay(pt);
	cv::Point3d &normal = m_plane.normal;
	cv::Point3d &plane  = m_plane.point;
	return (plane.dot(normal) / ray.dot(normal)) * ray;
}

void StereoObstacleDetection::PointCloudCallback(PointCloud::ConstPtr const &pc_xyz)
{
	ROS_ASSERT(pc_xyz.is_dense);

	// Unit vector that is orthgonal to the plane, points towards the robot, and
	// is normalized to have unit length.
	cv::Vec3d normal = m_plane.normal / cv::norm(m_plane.normal);
	if (normal[1] < 0.0) {
		normal = -1.0 * normal;
	}

	// Vector parallel to the image plane. Later scaled by the cone's height.
	cv::Vec3d side(0.0, cos(m_theta), 0.0);

	cv::Mat  segments(pc_xyz.height, pc_xyz.width, CV_32U, cv::Scalar(0));
	uint32_t segment = 1;

	for (int y0 = pc_xyz.height; y0 >= 0; --y0)
	for (int x0 = pc_xyz.width;  x0 >= 0; --x0) {
		cv::Point2d pt2(x0, y0);

		// Three-dimensional coordinates of the inverted cone.
		cv::Point3d pt3    = GetGroundPoint(pt2);
		cv::Point3d pt3_bl = pt + (normal - side) * m_minz;
		cv::Point3d pt3_br = pt + (normal + side) * m_minz;
		cv::Point3d pt3_tl = pt + (normal - side) * m_maxz;
		cv::Point3d pt3_tr = pt + (normal + side) * m_maxz;

		// Projection of the inverted cone into the image.
		cv::Point2d pt2_bl = model.project3dToPixel(pt3_bl);
		cv::Point2d pt2_br = model.project3dToPixel(pt3_br);
		cv::Point2d pt2_tl = model.project3dToPixel(pt3_tl);
		cv::Point2d pt2_tr = model.project3dToPixel(pt3_tr);

		double radius_bot = 0.5 * cv::norm(pt2_br - pt2_bl);
		double radius_top = 0.5 * cv::norm(pt2_tr - pt2_tl);

		// TODO: Search the upper trapezoid for compatible points.
		// TODO: Assign all compatible points the same label.
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "od_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	m_sub_tf = new tf::TransformListener;
	m_pub_tf = new tf::TransformBroadcaster;

	nh_priv.param<double>("minz", m_minz, 0.100);
	nh_priv.param<double>("maxz", m_maxz, 2.000);
	nh_priv.param<double>("theta", m_theta, M_PI / 2);
	nh_priv.param<std::string>("plane", m_plane, "/ground_plane");

	m_sub_pts = nh.subscribe<PointCloud>("stereo_points", 1, &PointCloudCallback);

	ros::spin();

	delete m_sub_tf;
	delete m_pub_tf;
}

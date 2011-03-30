#include "LineTrackerNode.hpp"

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>

LineTrackerNode::LineTrackerNode(ros::NodeHandle nh, int inliers)
	: m_nh(nh)
{
	m_inliers = inliers;
	m_sub_pts     = nh.subscribe<PointNormalCloud>("line_points", 1, &LineTrackerNode::PointCloudCallback, this);
	m_pub_inliers = m_nh.advertise<PointCloud>("line_inliers", 10);
}

void LineTrackerNode::PointCloudCallback(PointNormalCloud::ConstPtr const &msg_ptsn)
{
	// Discard the points' normal vector (for now).
	// TODO: Can I do this without copying any memory?
	PointCloud msg_pts;
	msg_pts.points.resize(msg_ptsn->points.size());

	for (size_t i = 0; i < msg_ptsn->points.size(); ++i) {
		pcl::PointNormal  pt_src = msg_ptsn->points[i];
		pcl::PointXYZ    &pt_dst = msg_pts.points[i];
		pt_dst.x = pt_src.x;
		pt_dst.y = pt_src.y;
		pt_dst.z = pt_src.z;
	}

	// Use RANSAC to fit as many linear models as possible.
	std::vector<pcl::PointXYZ> pts_inliers;
	std::vector<pcl::PointXYZ> pts_outliers;

	for (size_t i = 0; i < msg_pts.points.size(); ++i) {
		pts_outliers.push_back(msg_pts.points[i]);
	}

	int fits = 0;
	for (;;) {
		// Update the faux message used to satisfy PCL.
		// TODO: This really should not be necessary.
		msg_pts.points.clear();
		for (size_t i = 0; i < pts_outliers.size(); ++i) {
			msg_pts.points.push_back(pts_outliers[i]);
		}

		// Fit another linear model to the remaining outliers.
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.010);
		seg.setInputCloud(msg_pts.makeShared());

		pcl::ModelCoefficients coefficients;
		pcl::PointIndices inliers;
		seg.segment(inliers, coefficients);

		// Update the inlier and outlier sets for the next fit.
		pts_outliers.clear();

		if ((int)inliers.indices.size() >= m_inliers) {
			// TODO: Do something with the model.
			++fits;

			for (size_t i = 0; i < msg_pts.points.size(); ++i) {
				bool is_inlier = false;
				for (size_t j = 0; j < inliers.indices.size(); ++j) {
					if ((int)i == inliers.indices[j]) {
						is_inlier = true;
						break;
					}
				}

				if (is_inlier) {
					pts_inliers.push_back(msg_pts.points[i]);
				} else {
					pts_outliers.push_back(msg_pts.points[i]);
				}
			}
		} else break;
	}

	// Publish the inliers.
	PointCloud::Ptr msg_inliers(new PointCloud);
	msg_inliers->header.stamp    = msg_ptsn->header.stamp;
	msg_inliers->header.frame_id = msg_ptsn->header.frame_id;
	msg_inliers->height = 1;
	msg_inliers->width  = pts_inliers.size();

	for (size_t i = 0; i < pts_inliers.size(); ++i) {
		msg_inliers->points.push_back(pts_inliers[i]);
	}

	m_pub_inliers.publish(msg_inliers);
	ROS_INFO("Fit %d lines to %d inliers.", fits, (int)msg_inliers->points.size());

	// TODO: Publish the PWL fit as visualization markers.
}

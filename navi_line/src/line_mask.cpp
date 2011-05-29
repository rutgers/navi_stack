#include "line_mask.hpp"

namespace navi_line {

LineMaskNode::LineMaskNode(void)
	: nh_priv("~")
{}

void LineMaskNode::onInit(void)
{
	m_tf  = boost::make_shared<tf::TransformListener>(nh, ros::Duration(1.0));
	m_map = boost::make_shared<Costmap2DROS>("obstacles", *m_tf);
}

};

#ifndef LINE_MASK_HPP_
#define LINE_MASK_HPP_

namespace navi_line {

using costmap_2d::Costmap2DROS;

class LineMaskNode {
public:
	LineMaskNode(void);
	virtual void onInit(void);

private:
	ros::NodeHandle nh, nh_priv;

	boost::shared_ptr<Costmap2DROS> m_costmap;
};

};

#endif

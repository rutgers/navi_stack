#ifndef ASTAR_H_
#define ASTAR_H_

#include <algorithm>
#include <vector>
#include <boost/multi_array.hpp>

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

namespace navi_astar {
struct Node {
    unsigned int const x, y;
    double const path_cost;

    Node(unsigned int x, unsigned int y, double path_cost);
    bool operator==(Node const &other);
    bool operator!=(Node const &other);
};

struct Predecessor {
    Node node;
    double cost;
};

class AStarPlanner : public nav_core::BaseGlobalPlanner {
public:
    typedef boost::multi_array<uint8_t, 2> Array2;
    typedef boost::multi_array<Node, 2> PredecessorArray;
    typedef boost::shared_ptr<Array2> Array2Ptr;

    static uint8_t const kCostObstacle;
    static uint8_t const kCostUnknown;

    AStarPlanner(void);
    AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    virtual ~AStarPlanner(void);

    // Search Algorithm
    bool search(double start_x, double start_y,
                double goal_x, double goal_y);

    inline bool isInBounds(Node const &node)
    {
        unsigned int const width = costmap_ros_->getSizeInCellsX();
        unsigned int const height = costmap_ros_->getSizeInCellsX();

        return (0 <= node.x && node.x < width)
            && (0 <= node.y && node.y < height);
    }

    // Distance Transform
    Array2Ptr getBinaryCostmap(costmap_2d::Costmap2D const &costmap);
    void distanceTransform(costmap_2d::Costmap2D const &costmap, Array2 &binary);

    // Visualization
    void visualizeDistance(costmap_2d::Costmap2D const &costmap, Array2 const &distances);

    // BaseGlobalPlanner interface
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(geometry_msgs::PoseStamped const &start,
                  geometry_msgs::PoseStamped const &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);
    void publishPlan(std::vector<geometry_msgs::PoseStamped> const& path,
                     double r, double g, double b, double a);
    bool makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

private:
    costmap_2d::Costmap2DROS *costmap_ros_;
    bool initialized_;
    double distance_max_;

    pcl_ros::Publisher<pcl::PointXYZI> pub_distances_;
    ros::Publisher pub_plan_;

    double inscribed_radius_, circumscribed_radius_, inflation_radius_;

private:
    inline double sq_distance(geometry_msgs::PoseStamped const& p1,
                              geometry_msgs::PoseStamped const& p2)
    {
        double dx = p1.pose.position.x - p2.pose.position.x;
        double dy = p1.pose.position.y - p2.pose.position.y;
        return dx*dx +dy*dy;
    }
};
};

#endif

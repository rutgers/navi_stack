#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <navi_astar/astar.h>

PLUGINLIB_DECLARE_CLASS(navi_astar, AStarPlanner, navi_astar::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace navi_astar {

uint8_t const AStarPlanner::kCostObstacle = 253;
uint8_t const AStarPlanner::kCostUnknown  = 255;

/*
 * Node Datastructure
 */
Node::Node(unsigned int x, unsigned int y)
    : x(x), y(y)
{
}

bool Node::operator==(Node const &other) const
{
    return x == other.x && y == other.y;
}

bool Node::operator!=(Node const &other) const
{
    return !(*this == other);
}

Predecessor::Predecessor(Node node, double cost)
    : node(node), cost(cost)
{
}

bool Predecessor::operator<(Predecessor const &other) const
{
    return cost < other.cost;
}

/*
 * A* Planner
 */

AStarPlanner::AStarPlanner(void)
    : initialized_(false)
{
    ROS_INFO("Constructed A* Planner");
}

AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : initialized_(false)
{
    ROS_INFO("Constructed A* Planner");
    initialize(name, costmap_ros);
}

AStarPlanner::~AStarPlanner(void)
{}

void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle nh_priv("~/" + name);
    pub_distances_.advertise(nh_priv, "distances", 1);
    nh_priv.param("max_distance", distance_max_, 2.0);
    costmap_ros_ = costmap_ros;
    initialized_ = true;

    ROS_INFO("Initialized A* Planner");
}

AStarPlanner::Array2Ptr AStarPlanner::getBinaryCostmap(costmap_2d::Costmap2D const &costmap)
{
    uint8_t const *raw = costmap.getCharMap();

    Array2Ptr binary = boost::make_shared<Array2>(boost::extents[height_][width_]);

    for (unsigned int y = 0; y < height_; ++y)
    for (unsigned int x = 0; x < width_; ++x) {
        uint8_t const cost = raw[y * height_ + x];

        // Initialize the cell to be proportional to the distance from the
        // obstacle.
        if (cost == kCostUnknown || cost < kCostObstacle) {
            (*binary)[y][x] = std::numeric_limits<uint8_t>::max();
        } else {
            (*binary)[y][x] = std::numeric_limits<uint8_t>::min();
        }
    }
    return binary;
}

void AStarPlanner::distanceTransform(costmap_2d::Costmap2D const &costmap, Array2 &binary)
{
    bool changed;
    int iteration = 0;

    int const max_iterations = static_cast<int>(ceil(distance_max_ / resolution_));

    do {
        changed = false;

        for (unsigned int y = min_y_; y < max_y_; ++y)
        for (unsigned int x = min_x_; x < max_x_; ++x) {
            uint8_t &value = binary[y][x];
            unsigned int const value_old = value;

            value = (uint8_t)std::min((unsigned int)value, binary[y - 1][x] + 1u);
            value = (uint8_t)std::min((unsigned int)value, binary[y + 1][x] + 1u);
            value = (uint8_t)std::min((unsigned int)value, binary[y][x - 1] + 1u);
            value = (uint8_t)std::min((unsigned int)value, binary[y][x + 1] + 1u);

            // This is a fixed point algorithm that terminates when the costs
            // stop changing.
            changed = changed || (value < value_old);
        }
        iteration++;
    } while (changed && iteration < max_iterations);
}

void AStarPlanner::visualizeDistance(costmap_2d::Costmap2D const &costmap,
                                     Array2 const &distances)
{
    double const origin_x = costmap.getOriginX();
    double const origin_y = costmap.getOriginY();

    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = costmap_ros_->getGlobalFrameID();

    for (unsigned int y = min_y_; y < max_y_; ++y)
    for (unsigned int x = min_x_; x < max_x_; ++x) {
        double const distance = distances[y][x] * resolution_;

        pcl::PointXYZI pt;
        pt.x = resolution_ * x + origin_x;
        pt.y = resolution_ * y + origin_y;
        pt.z = 0.0;
        pt.intensity = distance;
        cloud.push_back(pt);
    }

    ROS_INFO("A* Published %d points.", (int)cloud.size());
    pub_distances_.publish(cloud);
}

/*
 * Plan
 */
bool AStarPlanner::search(double start_x, double start_y,
                          double goal_x, double goal_y)
{
    //PredecessorArray predecessor(boost::extents[height_][width_]);
    //Array2 visited(boost::extents[height_][width_]);
    std::priority_queue<Predecessor> fringe;
    fringe.push(Predecessor(Node(start_x, start_y), 0.0));

#if 0

    std::fill(visited.origin(), visited.origin() + visited.size(), 0)

    Node const node_start(start_x, start_y, 0);
    Node const node_goal(goal_x, goal_y, 0);

    while (!fringe.empty) {
        Node const node = fringe.top();
        fringe.pop();

        if (node == node_goal) {
            ROS_INFO("Search Done");
            break;
        } else if (visited[node.y][node.x]) {
            continue;
        }

        visited[node.y][node.x] = true;

        double next_cost = node.path_cost + 1;
        Node const nl(node.x - 1, node.y);
        Node const nr(node.x + 1, node.y);
        Node const nt(node.x, node.y - 1);
        Node const nb(node.x, node.y + 1);

        if (isInBounds(nl) && nl < ) fringe.push(nl);
        if (isInBounds(nr)) fringe.push(nr);
        if (isInBounds(nt)) fringe.push(nt);
        if (isInBounds(nb)) fringe.push(nb);
    }
#endif
    return false;
}

/*
 * BaseGlobalPlanner interface
 */
bool AStarPlanner::makePlan(geometry_msgs::PoseStamped const &start,
                            geometry_msgs::PoseStamped const &goal,
                            std::vector<geometry_msgs::PoseStamped> &plan)
{
    ROS_INFO("A* Make Plan");
    costmap_2d::Costmap2D costmap;
    costmap_ros_->getCostmapCopy(costmap);
    width_  = costmap.getSizeInCellsX();
    height_ = costmap.getSizeInCellsY();
    resolution_ = costmap.getResolution();
    
    geometry_msgs::Point position = start.pose.position;
    unsigned int start_x, start_y;
    bool in_bounds = costmap.worldToMap(position.x, position.y,start_x, start_y);
    if (!in_bounds) {
        ROS_ERROR_THROTTLE(10, "Robot is outside the global costmap.");
        return false;
    }

    min_x_ = std::max(start_x - 20, 0u);
    max_x_ = std::min(start_x + 20, width_);
    min_y_ = std::max(start_y - 20, 0u);
    max_y_ = std::min(start_y + 20, height_);

    ROS_INFO("%d %d %d %d", min_x_, max_x_, min_y_, max_y_);

    Array2Ptr distances = getBinaryCostmap(costmap);
    distanceTransform(costmap, *distances);
    visualizeDistance(costmap, *distances);
    return true;
}

void AStarPlanner::publishPlan(std::vector<geometry_msgs::PoseStamped> const& path,
                               double r, double g, double b, double a)
{
    
}

bool AStarPlanner::makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
{
    return true;
}

};

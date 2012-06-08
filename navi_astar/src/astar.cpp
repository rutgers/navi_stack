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

Predecessor::Predecessor(void)
    : node(0, 0), initialized(false)
{
}

Predecessor::Predecessor(Node node, double cost_path, double cost_heuristic)
    : node(node)
    , cost_path(cost_path)
    , cost_heuristic(cost_heuristic)
    , initialized(true)
{
}

bool Predecessor::operator<(Predecessor const &other) const
{
    return cost_path + cost_heuristic < other.cost_path + other.cost_heuristic;
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
bool AStarPlanner::search(Node const &node_start, Node const &node_goal)
{
    BinaryArray visited(boost::extents[height_][width_]);
    std::fill(visited.origin(), visited.origin() + visited.size(), false);

    PredecessorArray predecessor(boost::extents[height_][width_]);
    std::priority_queue<Predecessor> fringe;

    double start_heuristic = getHeuristicValue(node_start, node_goal);
    Predecessor predecessor_start(node_start, 0.0, start_heuristic);
    fringe.push(predecessor_start);

    while (!fringe.empty()) {
        Predecessor const current = fringe.top();
        Node const node = current.node;
        fringe.pop();

        ROS_INFO("Cost = %f", current.cost_path);

        if (node == node_goal) {
            ROS_INFO("Search Done");
            break;
        } else if (visited[node.y][node.x]) {
            continue;
        } else {
            visited[node.y][node.x] = true;
        }

        if (node.x > 0)
            fringe.push(getPredecessor(current, node_goal, -1,  0));
        if (node.x < width_ - 1)
            fringe.push(getPredecessor(current, node_goal, +1,  0));
        if (node.y > 0)
            fringe.push(getPredecessor(current, node_goal,  0, -1));
        if (node.y < height_ - 1)
            fringe.push(getPredecessor(current, node_goal, -1, +1));
    }
    return false;
}

Predecessor AStarPlanner::getPredecessor(Predecessor const &curr, Node const &goal,
                                         int dx, int dy)
{
    Node const node(curr.node.x + dx, curr.node.y + dy);

    // TODO: Use the weighted costs in this calculation.
    double const new_cost_path = curr.cost_path + 1.0;
    double const new_cost_heuristic = getHeuristicValue(curr.node, goal);

    return Predecessor(node, new_cost_path, new_cost_heuristic);
}

Node AStarPlanner::getNode(double world_x, double world_y)
{
    Node node(0, 0);
    // TODO: Check if it's in bounds.
    costmap_.worldToMap(world_x, world_y, node.x, node.y);
    return node;
}

double AStarPlanner::getHeuristicValue(Node const &node, Node const &goal)
{
    return resolution_ * sqrt(pow(node.x - goal.x, 2) + pow(node.y - goal.y, 2));
}

/*
 * BaseGlobalPlanner interface
 */
bool AStarPlanner::makePlan(geometry_msgs::PoseStamped const &start,
                            geometry_msgs::PoseStamped const &goal,
                            std::vector<geometry_msgs::PoseStamped> &plan)
{
    ROS_INFO("A* Make Plan");
    costmap_ros_->getCostmapCopy(costmap_);
    width_  = costmap_.getSizeInCellsX();
    height_ = costmap_.getSizeInCellsY();
    resolution_ = costmap_.getResolution();
    
    geometry_msgs::Point position = start.pose.position;
    unsigned int start_x, start_y;
    bool in_bounds = costmap_.worldToMap(position.x, position.y,start_x, start_y);
    if (!in_bounds) {
        ROS_ERROR_THROTTLE(10, "Robot is outside the global costmap.");
        return false;
    }

    min_x_ = std::max(start_x - 20, 0u);
    max_x_ = std::min(start_x + 20, width_);
    min_y_ = std::max(start_y - 20, 0u);
    max_y_ = std::min(start_y + 20, height_);

    ROS_INFO("%d %d %d %d", min_x_, max_x_, min_y_, max_y_);

    Array2Ptr distances = getBinaryCostmap(costmap_);
    distanceTransform(costmap_, *distances);

    geometry_msgs::Point position_start = start.pose.position;
    geometry_msgs::Point position_goal = goal.pose.position;
    Node const node_start = getNode(position_start.x, position_start.y);
    Node const node_goal = getNode(position_goal.x, position_goal.y);
    search(node_start, node_goal);

    visualizeDistance(costmap_, *distances);
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

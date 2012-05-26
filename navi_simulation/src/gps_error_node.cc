#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>

typedef boost::mt19937 RNGType;
typedef boost::normal_distribution<> normal_dist;
typedef boost::variate_generator<RNGType, normal_dist> normal_generator;

static double const big = 99999;

static ros::Subscriber sub_odom;
static ros::Publisher pub_gps;

static std::string frame_id, child_frame_id;

static double sigma_x, sigma_y;
static RNGType rng;
static boost::shared_ptr<normal_generator> noise_x, noise_y;

static ros::Time last_publish;
static ros::Duration rate_period;

void updateGPS(nav_msgs::Odometry const &msg_in)
{
    // Throttle the publishing rate.
    if (msg_in.header.stamp - last_publish < rate_period) return;

    // Republish a noisy odometry message.
    nav_msgs::Odometry msg_out;
    msg_out.header.stamp = msg_in.header.stamp;

    if (msg_in.header.frame_id != "" && msg_in.header.frame_id != frame_id) {
        ROS_WARN_THROTTLE(10, "changing frame_id from '%s' to '%s'",
            msg_in.header.frame_id.c_str(), frame_id.c_str());
    }
    if (msg_in.child_frame_id != "" && msg_in.child_frame_id != child_frame_id) {
        ROS_WARN_THROTTLE(10, "changing child_frame_id from '%s' to '%s'",
            msg_in.child_frame_id.c_str(), child_frame_id.c_str());
    }
    msg_out.header.frame_id = frame_id;
    msg_out.child_frame_id = child_frame_id;
    msg_out.pose.pose.position.x = msg_in.pose.pose.position.x + (*noise_x)();
    msg_out.pose.pose.position.y = msg_in.pose.pose.position.y + (*noise_y)();

    double const varx = pow(sigma_x, 2);
    double const vary = pow(sigma_y, 2);
    Eigen::Map<Eigen::Matrix<double, 6, 6> > cov_raw(&msg_out.pose.covariance[0]);
    cov_raw << varx, 0.0,  0.0, 0.0, 0.0, 0.0,
               0.0,  vary, 0.0, 0.0, 0.0, 0.0,
               0.0,  0.0,  big, 0.0, 0.0, 0.0,
               0.0,  0.0,  0.0, big, 0.0, 0.0,
               0.0,  0.0,  0.0, 0.0, big, 0.0,
               0.0,  0.0,  0.0, 0.0, 0.0, big;
    msg_out.twist.covariance[0] = -1;

    pub_gps.publish(msg_out);
    last_publish = msg_in.header.stamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_error_node");

    ros::NodeHandle nh, nh_node("~");
    double rate_hz;
    nh_node.param<std::string>("frame_id", frame_id, "/map");
    nh_node.param<std::string>("child_frame_id", child_frame_id, "/odom");
    nh_node.param<double>("sigma_x", sigma_x, 1e-6);
    nh_node.param<double>("sigma_y", sigma_y, 1e-6);
    nh_node.param<double>("rate", rate_hz, 20.0);

    ROS_WARN("sigma_x = %f m", sigma_x);
    ROS_WARN("sigma_y = %f m", sigma_y);
    ROS_WARN("rate = %f Hz", rate_hz);

    rate_period  = ros::Duration(1 / rate_hz);
    last_publish = ros::Time(0);

    // Gaussian random number generators to add noise.
    normal_dist const dist_x(0.0, sigma_x);
    normal_dist const dist_y(0.0, sigma_y);
    noise_x = boost::make_shared<normal_generator>(rng, dist_x);
    noise_y = boost::make_shared<normal_generator>(rng, dist_y);

    sub_odom = nh.subscribe("ground_truth", 1, &updateGPS);
    pub_gps  = nh.advertise<nav_msgs::Odometry>("gps", 10);

    ros::spin();
    return 0;
}

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>

typedef boost::normal_distribution<> normal_dist;
typedef boost::mt19937 RNGType;

static ros::Subscriber sub_odom;
static boost::shared_ptr<tf::TransformBroadcaster> pub_tf;
static ros::Publisher pub_odom;

static Eigen::Vector3d last_pos, last_noisy_pos;
static double last_angle, last_noisy_angle;

static double robot_radius;
static double alpha;
static double const min_variance = 1e-6;
static RNGType rng;

void updateOdom(nav_msgs::Odometry const &msg_in)
{
    // Change in position and orientation measured since the last message.
    Eigen::Vector3d const curr_pos = (Eigen::Vector3d()
        << msg_in.pose.pose.position.x
        ,  msg_in.pose.pose.position.y
        ,  msg_in.pose.pose.position.z).finished();
    double const curr_angle = tf::getYaw(msg_in.pose.pose.orientation);

    // Convert the changes into polar coordinates.
    Eigen::Vector3d const delta_pos = curr_pos - last_pos;
    Eigen::Vector3d const u = (Eigen::Vector3d() <<
        cos(curr_angle), sin(curr_angle), 0.0).finished();
    double const delta_linear = delta_pos.transpose() * u;
    double const delta_angle  = angles::normalize_angle(curr_angle - last_angle);

    // Convert from polar coordinates to encoder ticks.
    double const ticks_left  = delta_linear - 0.5 * robot_radius * delta_angle;
    double const ticks_right = delta_linear + 0.5 * robot_radius * delta_angle;

    // Add noise to the encoder ticks.
    double const sigma_left  = std::max(alpha * ticks_left,  min_variance);
    double const sigma_right = std::max(alpha * ticks_right, min_variance);
    normal_dist const dist_left(0.0, sigma_left);
    normal_dist const dist_right(0.0, sigma_right);
    boost::variate_generator<RNGType, normal_dist> noise_left(rng, dist_left);
    boost::variate_generator<RNGType, normal_dist> noise_right(rng, dist_right);
    double const noisy_ticks_left  = ticks_left + noise_left();
    double const noisy_ticks_right = ticks_right + noise_right();

    // Convert back from encoder ticks to polar coordinates.
    double const noisy_delta_linear = 0.5 * (noisy_ticks_left + noisy_ticks_right);
    double const noisy_delta_angle  = (noisy_ticks_right - noisy_ticks_left) / robot_radius;

    // Convert back from polar coordinates to cartaesian coordinates.
    double const noisy_angle = angles::normalize_angle(last_noisy_angle + noisy_delta_angle);
    Eigen::Vector3d const noisy_pos = (Eigen::Vector3d()
        << last_noisy_pos[0] + noisy_delta_linear * cos(noisy_angle)
        ,  last_noisy_pos[1] + noisy_delta_linear * sin(noisy_angle)
        ,  0.0).finished();

    // Generate an odometry message from the noisy measurements.
    nav_msgs::Odometry msg_out;
    msg_out.header = msg_in.header;

    // TODO: Check if child_frame_id is set before overrwrite it.
    msg_out.child_frame_id = "/base_footprint";

    msg_out.pose.pose.position.x = noisy_pos[0];
    msg_out.pose.pose.position.y = noisy_pos[1];
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(noisy_angle),
                          msg_out.pose.pose.orientation);
    msg_out.twist.covariance[0] = -1;

    // TODO: Propagate the variances through the transformation.
#if 0
    double const var_x = sigma_linear * sigma_linear * cos(last_angle);
    double const var_y = sigma_linear * sigma_linear * sin(last_angle);
    double const var_angle = sigma_angular * sigma_angular;
#else
    double const var_x = 0.0;
    double const var_y = 0.0;
    double const var_angle = 0.0;
#endif
    double const big = 99999;
    Eigen::Map<Eigen::Matrix<double, 6, 6> > cov_raw(&msg_out.pose.covariance[0]);
    cov_raw << var_x,   0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, var_y, 0.0, 0.0, 0.0, 0.0,
                 0.0,   0.0, big, 0.0, 0.0, 0.0,
                 0.0,   0.0, 0.0, big, 0.0, 0.0,
                 0.0,   0.0, 0.0, 0.0, big, 0.0,
                 0.0,   0.0, 0.0, 0.0, 0.0, var_angle;

    // TODO: Also publish a TF transform.
    pub_odom.publish(msg_out);

    last_pos = curr_pos;
    last_angle = curr_angle;
    last_noisy_pos = noisy_pos;
    last_noisy_angle = noisy_angle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_error_node");

    last_pos << 0.0, 0.0, 0.0;
    last_angle =  0.0;

    alpha = 0.1;
    robot_radius = 0.3;
    pub_tf = boost::make_shared<tf::TransformBroadcaster>();

    ros::NodeHandle nh;
    sub_odom = nh.subscribe("odom_true", 1, &updateOdom);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);

    ros::spin();
    return 0;
}

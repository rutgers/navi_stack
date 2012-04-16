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
static tf::TransformBroadcaster pub_tf;
static ros::Publisher pub_odom;

static Eigen::Vector3d last_pos;
static double last_angle;

static double alpha_linear;
static double alpha_angular;
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
    double const delta_angle = angles::normalize_angle(curr_angle - last_angle);

    // Add Gaussian noise to the polar coordinates. Noise on the linear and
    // angular components are drawn from independent Gaussian distributions
    // with standard deviation proportional to velocity.
    // TODO: Account for the time difference between the two messages.
    double const sigma_linear = alpha_linear * delta_pos.norm();
    double const sigma_angular = alpha_angular * fabs(delta_angle);
    normal_dist const dist_linear(delta_pos.norm(), sigma_linear);
    normal_dist const dist_angular(delta_angle, sigma_angular);
    boost::variate_generator<RNGType, normal_dist> gen_linear(rng, dist_linear);
    boost::variate_generator<RNGType, normal_dist> gen_angular(rng, dist_angular);
    double const noisy_delta_linear = gen_linear();
    double const noisy_delta_angle  = gen_angular();

    // Transform back from polar to cartaesian coordinates. Also update our
    // internal state for the next message.
    Eigen::Vector3d const noisy_pos = (Eigen::Vector3d()
        << last_pos[0] + noisy_delta_linear * cos(last_angle)
        ,  last_pos[1] + noisy_delta_linear * sin(last_angle)
        ,  0.0).finished();
    double const noisy_angle = angles::normalize_angle(curr_angle + noisy_delta_angle);
    last_pos = noisy_pos;
    last_angle = noisy_angle;

    // Generate an odometry message from the noisy measurements.
    nav_msgs::Odometry msg_out;
    msg_out.header = msg_in.header;
    msg_out.child_frame_id = msg_in.child_frame_id;
    msg_out.pose.pose.position.x = noisy_pos[0];
    msg_out.pose.pose.position.y = noisy_pos[1];
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(noisy_angle),
                          msg_out.pose.pose.orientation);
    msg_out.twist.covariance[0] = -1;

    double const var_x = sigma_linear * sigma_linear * cos(last_angle);
    double const var_y = sigma_linear * sigma_linear * sin(last_angle);
    double const var_angle = sigma_angular * sigma_angular;
    double const big = std::numeric_limits<double>::infinity();
    Eigen::Map<Eigen::Matrix<double, 6, 6> > cov_raw(&msg_out.pose.covariance[0]);
    cov_raw << var_x,   0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, var_y, 0.0, 0.0, 0.0, 0.0,
                 0.0,   0.0, big, 0.0, 0.0, 0.0,
                 0.0,   0.0, 0.0, big, 0.0, 0.0,
                 0.0,   0.0, 0.0, 0.0, big, 0.0,
                 0.0,   0.0, 0.0, 0.0, 0.0, var_angle;

    // TODO: Also publish a TF transform.
    pub_odom.publish(msg_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_kf_node");

    last_pos << 0.0, 0.0, 0.0;
    last_angle =  0.0;

    ros::NodeHandle nh;
    sub_odom = nh.subscribe("odom_true", 1, &updateOdom);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);

    ros::spin();
    return 0;
}

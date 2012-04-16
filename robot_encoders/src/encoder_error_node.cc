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

static int ticks_per_rev;
static double alpha;
static double robot_radius, wheel_radius;
static double const min_variance = 1e-6;
static std::string frame_id, child_frame_id;
static RNGType rng;

double add_discretization_error(double distance, int ticks_per_rev)
{
#if 0
    // TODO: Figure out why this discretization error doesn't work.  #if 0
    double const revs = distance / 2 * M_PI * wheel_radius;
    int const ticks = round(revs * ticks_per_rev);
    double const measured_revs = static_cast<double>(ticks) / ticks_per_rev;
    return measured_revs * 2 * M_PI * wheel_radius;
#endif
    return distance;
}

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
    double const exact_v_left  = delta_linear - 0.5 * robot_radius * delta_angle;
    double const exact_v_right = delta_linear + 0.5 * robot_radius * delta_angle;
    double const v_left  = add_discretization_error(exact_v_left, ticks_per_rev);
    double const v_right = add_discretization_error(exact_v_right, ticks_per_rev);

    // Add noise to the encoder ticks.
    double const sigma_left  = std::max(alpha * v_left,  min_variance);
    double const sigma_right = std::max(alpha * v_right, min_variance);
    normal_dist const dist_left(0.0, sigma_left);
    normal_dist const dist_right(0.0, sigma_right);
    boost::variate_generator<RNGType, normal_dist> noise_left(rng, dist_left);
    boost::variate_generator<RNGType, normal_dist> noise_right(rng, dist_right);
    double const noisy_v_left  = v_left + noise_left();
    double const noisy_v_right = v_right + noise_right();

    // Convert back from encoder ticks to polar coordinates.
    double const noisy_delta_linear = 0.5 * (noisy_v_left + noisy_v_right);
    double const noisy_delta_angle  = (noisy_v_right - noisy_v_left) / robot_radius;

    // Convert back from polar coordinates to cartaesian coordinates.
    double const noisy_angle = angles::normalize_angle(last_noisy_angle + noisy_delta_angle);
    Eigen::Vector3d const noisy_pos = (Eigen::Vector3d()
        << last_noisy_pos[0] + noisy_delta_linear * cos(noisy_angle)
        ,  last_noisy_pos[1] + noisy_delta_linear * sin(noisy_angle)
        ,  0.0).finished();

    // Generate an odometry message from the noisy measurements.
    nav_msgs::Odometry msg_out;
    msg_out.header = msg_in.header;

    if (msg_in.header.frame_id != "" && msg_in.header.frame_id != frame_id) {
        ROS_WARN_THROTTLE(10, "changing frame_id from '%s' to '%s'",
            msg_in.header.frame_id.c_str(), frame_id.c_str());
    }
    if (msg_in.child_frame_id != "" && msg_in.child_frame_id != child_frame_id) {
        ROS_WARN_THROTTLE(10, "changing child_frame_id from '%s' to '%s'",
            msg_in.child_frame_id.c_str(), child_frame_id.c_str());
    }
    msg_out.header.frame_id = frame_id;
    msg_out.child_frame_id  = child_frame_id;

    msg_out.pose.pose.position.x = noisy_pos[0];
    msg_out.pose.pose.position.y = noisy_pos[1];
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(noisy_angle),
                          msg_out.pose.pose.orientation);
    msg_out.twist.covariance[0] = -1;

    // Propagate the variances through the transformation.
    double const var = 0.5 * (pow(sigma_left, 2) + pow(sigma_right, 2));
    double const big = 99999;
    Eigen::Map<Eigen::Matrix<double, 6, 6> > cov_raw(&msg_out.pose.covariance[0]);
    cov_raw << var, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, var, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, big, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, big, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, big, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, var;
    pub_odom.publish(msg_out);

    // Also publish a TF transform.
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(noisy_pos[0], noisy_pos[1], 0.0));
    transform.setRotation(tf::createQuaternionFromYaw(noisy_angle));
    pub_tf->sendTransform(tf::StampedTransform(
        transform,
        msg_in.header.stamp,
        child_frame_id, frame_id
    ));

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

    ros::NodeHandle nh;
    ros::param::get("~alpha", alpha);
    ros::param::get("~robot_radius", robot_radius);
    ros::param::get("~wheel_radius", wheel_radius);
    ros::param::get("~ticks_per_rev", ticks_per_rev);
    ros::param::get("~frame_id", frame_id);
    ros::param::get("~child_frame_id", child_frame_id);

    pub_tf = boost::make_shared<tf::TransformBroadcaster>();
    sub_odom = nh.subscribe("ground_truth", 1, &updateOdom);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);

    ros::spin();
    return 0;
}

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
#include <sensor_msgs/Imu.h>

typedef boost::mt19937 RNGType;
typedef boost::normal_distribution<> normal_dist;
typedef boost::variate_generator<RNGType, normal_dist> normal_generator;

static double const big = 99999;

static ros::Subscriber sub_odom;
static ros::Publisher pub_compass;

static std::string frame_id;
static double sigma;
static RNGType rng;
static boost::shared_ptr<normal_generator> noise;

static ros::Time last_publish;
static ros::Duration rate_period;

void updateCompass(nav_msgs::Odometry const &msg_in)
{
    // Throttle the publishing rate.
    if (msg_in.header.stamp - last_publish < rate_period) return;

    // Add noise
    double const var = pow(sigma, 2);
    double const angle = tf::getYaw(msg_in.pose.pose.orientation);
    double const noisy_angle = angles::normalize_angle(angle + (*noise)());

    // ...
    sensor_msgs::Imu msg_out;
    msg_out.header.stamp = msg_in.header.stamp;

    if (msg_in.header.frame_id != "" && msg_in.header.frame_id != frame_id) {
        ROS_WARN_THROTTLE(10, "changing frame_id from '%s' to '%s'",
            msg_in.header.frame_id.c_str(), frame_id.c_str());
    }

    msg_out.header.frame_id = frame_id;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(noisy_angle), msg_out.orientation);
    Eigen::Map<Eigen::Matrix3d> cov_raw(&msg_out.orientation_covariance[0]);
    cov_raw << big, 0.0, 0.0,
               0.0, big, 0.0,
               0.0, 0.0, var;

    pub_compass.publish(msg_out);
    last_publish = msg_in.header.stamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "compass_error_node");

    double rate_hz;
    ros::NodeHandle nh, nh_node("~");
    nh_node.param<std::string>("frame_id", frame_id, "/map");
    nh_node.param<double>("sigma", sigma, 1e-6);
    nh_node.param<double>("rate", rate_hz, 20.0);

    rate_period  = ros::Duration(1 / rate_hz);
    last_publish = ros::Time(0);

    // Gaussian random number generators to add noise.
    normal_dist const dist_noise(0.0, sigma);
    noise = boost::make_shared<normal_generator>(rng, dist_noise);

    sub_odom    = nh.subscribe("ground_truth", 1, &updateCompass);
    pub_compass = nh.advertise<sensor_msgs::Imu>("compass", 10);

    ros::spin();
    return 0;
}

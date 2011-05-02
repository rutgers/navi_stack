#include <ros/ros.h>
#include <joy/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Teleop
{
public:
  Teleop();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;


int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  

};

Teleop::Teleop():
  nh_("~"),
  linear_(1),
  angular_(2),
  l_scale_(0.5),
  a_scale_(2.0)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  ros::NodeHandle nh;
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh.subscribe<joy::Joy>("joy", 10, &Teleop::joyCallback, this);
}

void Teleop::joyCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist cmdvel;
  
      cmdvel.linear.x = l_scale_ * joy->axes[linear_];
      cmdvel.angular.z = a_scale_ * joy->axes[angular_];
  
  vel_pub_.publish(cmdvel);}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_gamepad_singlestick");
  Teleop teleop;

  ros::spin();
}

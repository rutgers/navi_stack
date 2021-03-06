/*
 * teleop_base_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7a
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_E 0x65
#define KEYCODE_C 0x63
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6d
#define KEYCODE_R 0x72
#define KEYCODE_V 0x76
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62

#define KEYCODE_COMMA 0x2c
#define KEYCODE_PERIOD 0x2e

#define COMMAND_TIMEOUT_SEC 0.2

// press down a key and you will initially go
double trans_vel = 0.3; // m/second
double angular_vel = 0.5*M_PI/180.0; // rad/second
// should we continuously send commands?
bool always_command = false;


class TBK_Node
{
  private:
    geometry_msgs::Twist cmdvel;
    ros::NodeHandle n_;
    ros::Publisher pub_;

  public:
    TBK_Node()
    {
      pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel",1);
    }
    ~TBK_Node() { }
    void keyboardLoop();
    void stopRobot()
    {
      cmdvel.linear.x = cmdvel.angular.z = 0.0;
      pub_.publish(cmdvel);
    }
};

TBK_Node* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int
main(int argc, char** argv)
{
  ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  

  
  ros::NodeHandle nh_("~");
  nh_.param("trans_vel", trans_vel, trans_vel);
  nh_.param("angular_vel", angular_vel, angular_vel);
  nh_.param("always_command", always_command, always_command);

  TBK_Node tbk;

  boost::thread t(boost::bind(&TBK_Node::keyboardLoop, &tbk));
  
  ros::spin();

  t.interrupt();
  t.join();
  tbk.stopRobot();
  tcsetattr(kfd, TCSANOW, &cooked);

  return(0);
}

void
TBK_Node::keyboardLoop()
{
  char c;
  double max_tv = trans_vel;
  double max_rv = angular_vel;
  bool dirty=false;

  int speed=0;
  int turn=0;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("q/z : increase/decrease max angular and linear speeds by 10%");
  puts("w/x : increase/decrease max linear speed by 10%");
  puts("e/c : increase/decrease max angular speed by 10%");
  puts("---------------------------");
  puts("Moving around:");
  puts("   u    i    o");
  puts("   j    k    l");
  puts("   m    ,    .");
  puts("anything else : stop");
  puts("---------------------------");

  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;
  for(;;)
  {
    boost::this_thread::interruption_point();
    
    // get the next event from the keyboard
    int num;
    if((num = poll(&ufd, 1, 250)) < 0)
    {
      perror("poll():");
      return;
    }
    else if(num > 0)
    {
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        return;
      }
    }
    else
      continue;

    switch(c)
    {
      case KEYCODE_I:
        speed = 1;
        turn = 0;
        dirty = true;
        break;
      case KEYCODE_K:
        speed = 0;
        turn = 0;
        dirty = true;
        break;
      case KEYCODE_O:
        speed = 1;
        turn = -1;
        dirty = true;
        break;
      case KEYCODE_J:
        speed = 0;
        turn = 1;
        dirty = true;
        break;
      case KEYCODE_L:
        speed = 0;
        turn = -1;
        dirty = true;
        break;
      case KEYCODE_U:
        turn = 1;
        speed = 1;
        dirty = true;
        break;
      case KEYCODE_COMMA:
        turn = 0;
        speed = -1;
        dirty = true;
        break;
      case KEYCODE_PERIOD:
        turn = 1;
        speed = -1;
        dirty = true;
        break;
      case KEYCODE_M:
        turn = -1;
        speed = -1;
        dirty = true;
        break;
      case KEYCODE_Q:
        max_tv += max_tv / 10.0;
        max_rv += max_rv / 10.0;
        if(always_command)
          dirty = true;
        break;
      case KEYCODE_Z:
        max_tv -= max_tv / 10.0;
        max_rv -= max_rv / 10.0;
        if(always_command)
          dirty = true;
        break;
      case KEYCODE_W:
        max_tv += max_tv / 10.0;
        if(always_command)
          dirty = true;
        break;
      case KEYCODE_X:
        max_tv -= max_tv / 10.0;
        if(always_command)
          dirty = true;
        break;
      case KEYCODE_E:
        max_rv += max_rv / 10.0;
        if(always_command)
          dirty = true;
        break;
    case KEYCODE_C:
        max_rv -= max_rv / 10.0;
        if(always_command)
          dirty = true;
        break;
    default:
      speed = 0;
      turn = 0;
      dirty = true;
    }
    if (dirty == true)
    {
      cmdvel.linear.x = speed * max_tv;
      cmdvel.angular.z = turn * max_rv;

      pub_.publish(cmdvel);
    }
  }
}

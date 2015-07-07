/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Evangelos Apostolidis
 *********************************************************************/

#include "pandora_teleop/motors_keyop.h"

namespace pandora_teleop
{
  struct termios cooked, raw;
  int kfd = 0;

  void quit(int sig)
  {
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
  }

  MotorsKeyop::MotorsKeyop(
    double max_linear,
    double max_angular)
  :
    linear_(0),
    angular_(0),
    linear_scale_(max_linear),
    angular_scale_(max_angular)
  {
    nh_.param("scale_linear", linear_scale_, linear_scale_);
    nh_.param("scale_angular", angular_scale_, angular_scale_);
    twist_pub_ = nh_.advertise < geometry_msgs::Twist > ("/cmd_vel", 1);
    pub_thread_ = boost::thread(&MotorsKeyop::publishTwist, this);
  }

  MotorsKeyop::~MotorsKeyop(void)
  {
    pub_thread_.join();
  }

  double MotorsKeyop::getLinearScale(void)
  {
    return linear_scale_;
  }

  double MotorsKeyop::getAngularScale(void)
  {
    return angular_scale_;
  }

  void MotorsKeyop::publishTwist(void)
  {
    geometry_msgs::Twist twist;
    ros::Rate rate(100);
    while (ros::ok)
    {
      twist.linear .x = linear_ * linear_scale_;
      twist.angular .z = angular_ * angular_scale_;
      {
        boost::mutex::scoped_lock lock(lock_);
        twist_pub_.publish(twist);
      }
      rate.sleep();
    }
  }

  void MotorsKeyop::keyLoop(void)
  {
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);

    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("---------------------");
    puts("Reading from keyboard");
    puts("---------------------");
    puts("\nUse arrow keys to move the robot:\n");

    while (true)
    {
      // get the next event from the keyboard
      if (read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }
      // ~ linear_=angular_=0;
      ROS_DEBUG("value: 0x%02X\n", c);
      switch (c)
      {
        case KEYCODE_LEFT:
          ROS_DEBUG("LEFT");
          angular_ += 0.1;
          dirty = true;
          break;
        case KEYCODE_A:
          ROS_DEBUG("LEFT_MAX");
          angular_ = 1.0;
          dirty = true;
          break;
        case KEYCODE_RIGHT:
          ROS_DEBUG("RIGHT");
          angular_ += -0.1;
          dirty = true;
          break;
        case KEYCODE_D:
          ROS_DEBUG("RIGHT_MAX");
          angular_ = -1.0;
          dirty = true;
          break;
        case KEYCODE_UP:
          ROS_DEBUG("UP");
          linear_ += 0.2;
          dirty = true;
          break;
        case KEYCODE_W:
          ROS_DEBUG("UP_MAX");
          linear_ = 1.0;
          dirty = true;
          break;
        case KEYCODE_DOWN:
          ROS_DEBUG("DOWN");
          linear_ += -0.2;
          dirty = true;
          break;
        case KEYCODE_S:
          ROS_DEBUG("DOWN_MAX");
          linear_ = -1.0;
          dirty = true;
          break;
        case KEYCODE_SPACE:
        case KEYCODE_H:
          ROS_DEBUG("HALT");
          linear_ = 0.0;
          angular_ = 0.0;
          dirty = true;
          break;
      }

      if (dirty == true)
      {
        if (linear_ > 1.0)
          linear_ = 1.0;
        if (linear_ < -1.0)
          linear_ = -1.0;
        if (angular_ > 1.0)
          angular_ = 1.0;
        if (angular_ < -1.0)
          angular_ = - 1.0;

        std::cout << "Linear  : " << linear_ * 100 << " % , "
          << linear_ * linear_scale_ << " m/s\n";
        std::cout << "Angular : " << angular_ * 100 << " % , "
          << angular_ * angular_scale_ << " r/s\n";
        std::cout << "########################" << '\n';

        dirty = false;
      }
    }
    return;
  }
}  // namespace pandora_teleop

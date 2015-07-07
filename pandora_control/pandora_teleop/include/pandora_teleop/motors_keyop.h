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
**********************************************************************/

#ifndef PANDORA_TELEOP_MOTORS_KEYOP_H
#define PANDORA_TELEOP_MOTORS_KEYOP_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>

#define KEYCODE_SPACE 0x20
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_H 'h'
#define KEYCODE_W 'w'
#define KEYCODE_A 'a'
#define KEYCODE_S 's'
#define KEYCODE_D 'd'

namespace pandora_teleop
{
  /**
  @brief Handles user interrupt signals
  @param sig [int] : signal number
  @return void
  **/
  void quit(int sig);

  /**
  @class MotorsKeyop
  @brief Controls the motors' velocity via keyboard commands from user
  **/
  class MotorsKeyop
  {
    public:
      /**
      @brief Default Contructor
      **/
      MotorsKeyop(double max_linear = 0.5, double max_angular = 0.8);

      /**
      @brief Default Destructor
      **/
      ~MotorsKeyop(void);

      /**
      @brief Getter for linear_scale_
      @return double
      **/
      double getLinearScale(void);

      /**
      @brief Getter for angular_scale_
      @return double
      **/
      double getAngularScale(void);

      /**
      @brief Publishes twist commands to motors' topic
      @return void
      **/
      void publishTwist(void);

      /**
      @brief Handles keyboard inputs and modifies the motors velocity accordingly
      @return void
      **/
      void keyLoop(void);

    private:
      ros ::NodeHandle nh_;  //!< ros node handle
      double linear_;  //!< linear motor velocity
      double linear_scale_;  //!< linear velocity scale
      double angular_;  //!< angular motor velocity
      double angular_scale_;  //!< angular velocity scale
      ros::Publisher twist_pub_;  //!< motors twist command publisher
      boost::thread pub_thread_;  //!< publisher thread
      boost::mutex lock_;  //!< mutex lock
  };
}  // namespace pandora_teleop
#endif  // PANDORA_TELEOP_MOTORS_KEYOP_H

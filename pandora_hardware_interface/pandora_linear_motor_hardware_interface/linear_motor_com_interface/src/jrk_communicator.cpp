/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Author: Vasilis Bosdelekidis
*********************************************************************/

#include <linear_motor_com_interface/jrk_com_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pololu_jrk_communicator_node");
  pandora_hardware_interface::linear::JrkComInterface
    pololu_jrk_driver("/dev/linear", 115200, 100);
  pololu_jrk_driver.init();

  int reply = pololu_jrk_driver.readFeedback();
  ROS_ERROR("Got this position feedback: %d", reply);
  reply = pololu_jrk_driver.readScaledFeedback();
  ROS_ERROR("Got this scaled feedback: %d", reply);
  reply = pololu_jrk_driver.readDutyCycle();
  ROS_ERROR("Got this speed: %d", reply);
  reply = pololu_jrk_driver.getErrors();
  ROS_ERROR("Got these errors: %d", reply);

  // Send 'target position' over the serial port
  pololu_jrk_driver.setTarget(132);
  reply = pololu_jrk_driver.readTarget();
  ROS_ERROR("Got this target: %d", reply);

  // Get feedback
  reply = pololu_jrk_driver.readFeedback();
  ROS_ERROR("Got this position feedback: %d", reply);
  reply = pololu_jrk_driver.readScaledFeedback();
  ROS_ERROR("Got this scaled feedback: %d", reply);
  pololu_jrk_driver.setTarget(3600);
  reply = pololu_jrk_driver.readTarget();
  ROS_ERROR("Got this target: %d", reply);

  // Get feedback
  reply = pololu_jrk_driver.readFeedback();
  ROS_ERROR("Got this position feedback: %d", reply);
  reply = pololu_jrk_driver.readScaledFeedback();
  ROS_ERROR("Got this scaled feedback: %d", reply);
  pololu_jrk_driver.closeDevice();
  ros::spinOnce();
}

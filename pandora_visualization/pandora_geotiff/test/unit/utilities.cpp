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
 * Authors:
 *   Chamzas Konstantinos <chamzask@gmail.com>
 *********************************************************************/

#include <vector>
#include <string>

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>

#include "pandora_geotiff/utilities.h"


namespace pandora_geotiff
{
  TEST(UtilitiesTest, sortPoses)
  {
    // Create random poses with times in descending order.
    std::vector<geometry_msgs::PoseStamped> sortedPoses;
    std::vector<geometry_msgs::PoseStamped> poses;
    std::vector<ros::Time> times;

    ros::Time::init();

    for (int i = 0; i < 10; i++)
    {
      geometry_msgs::PoseStamped tempPose;
      ros::Time tempTime;

      tempPose.pose.position.x = i;
      tempPose.pose.position.y = i;
      tempPose.pose.position.z = i;

      tempTime = ros::Time::now() + ros::Duration(600/(i + 1));

      poses.push_back(tempPose);
      times.push_back(tempTime);
    }

    sortedPoses = sortObjects(poses, times);

    ASSERT_EQ(sortedPoses[0].pose.position.x, 9);
    ASSERT_EQ(sortedPoses[0].pose.position.y, 9);
    ASSERT_EQ(sortedPoses[0].pose.position.z, 9);

    ASSERT_EQ(sortedPoses[9].pose.position.x, 0);
    ASSERT_EQ(sortedPoses[9].pose.position.y, 0);
    ASSERT_EQ(sortedPoses[9].pose.position.z, 0);
  }
}  // namespace pandora_geotiff

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utilities");
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

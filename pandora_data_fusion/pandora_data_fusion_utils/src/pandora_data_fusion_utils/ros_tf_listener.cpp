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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <string>

#include "pandora_data_fusion_utils/tf_listener.h"

namespace pandora_data_fusion
{
namespace pandora_data_fusion_utils
{

  RosTfListener::RosTfListener()
  {
    tf::StampedTransform tfTransform;

    waitForTransform("/world", "/map",
        ros::Time(0), ros::Duration(1));

    lookupTransform("/world", "/map",
        ros::Time(0), tfTransform);
  }

  bool RosTfListener::waitForTransform(const std::string& target_frame,
      const std::string& source_frame, const ros::Time& time,
      const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
      std::string* error_msg) const
  {
    bool flag;
    try
    {
      flag = listener.waitForTransform(target_frame, source_frame, time,
          timeout, polling_sleep_duration, error_msg);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[ALERT_HANDLER %d] %s", __LINE__, ex.what());
      throw TfException(
          "Something went wrong with tf, ignoring current message");
    }
    return flag;
  }

  void RosTfListener::lookupTransform(const std::string& target_frame,
      const std::string& source_frame, const ros::Time& time,
      tf::StampedTransform& transform) const
  {
    try
    {
      listener.lookupTransform(target_frame, source_frame, time, transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[ALERT_HANDLER %d] %s", __LINE__, ex.what());
      throw TfException(
          "Something went wrong with tf, ignoring current message");
    }
  }

}  // namespace pandora_data_fusion_utils
}  // namespace pandora_data_fusion

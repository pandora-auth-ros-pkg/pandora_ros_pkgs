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
#include <vector>
#include <boost/algorithm/string.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "pandora_alert_handler/objects/victim.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  Victim::Victim()
  {
    isTargeted_ = false;
    verified_ = false;
    valid_ = false;
    visited_ = false;
    timeFound_ = ros::Time::now();
    selectedObjectIndex_ = -1;
    probability_ = 0;
  }

  void Victim::getVisualization(visualization_msgs::MarkerArray* markers) const
  {
    visualization_msgs::Marker victimMarker;
    visualization_msgs::Marker victimDescription;
    victimMarker.header.frame_id = getGlobalFrame();
    victimDescription.header.frame_id = getGlobalFrame();
    victimMarker.header.stamp = ros::Time::now();
    victimDescription.header.stamp = ros::Time::now();
    victimMarker.ns = type_;
    victimDescription.ns = type_ + "_BRIEF";
    victimMarker.id = id_;
    victimDescription.id = id_;
    victimMarker.pose = pose_;
    victimDescription.pose = pose_;
    victimDescription.pose.position.z = pose_.position.z + 0.1;
    victimMarker.type = visualization_msgs::Marker::SPHERE;
    victimDescription.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    victimDescription.text = getFrameId();
    victimMarker.scale.x = 0.1;
    victimMarker.scale.y = 0.1;
    victimMarker.scale.z = 0.1;
    victimDescription.scale.z = 0.1;
    victimMarker.lifetime = ros::Duration(0.2);
    victimDescription.lifetime = ros::Duration(0.2);
    if (visited_)
    {
      boost::to_upper(victimDescription.text);
      victimMarker.color.r = 0;
      victimMarker.color.g = 0;
      victimMarker.color.b = 0;
      victimMarker.color.a = 0.7;
      victimDescription.color.r = 0;
      victimDescription.color.g = 0;
      victimDescription.color.b = 0;
      victimDescription.color.a = 1;
      if (valid_)
      {
        victimMarker.color.g = 1;
        victimDescription.color.g = 1;
        victimDescription.text = "VALID_" + victimDescription.text + "!";
      }
      else
      {
        if (verified_)
        {
          victimMarker.color.r = 1;
          victimDescription.color.r = 1;
          victimDescription.text = "REJECTED_" + victimDescription.text;
        }
        else
        {
          victimMarker.color.r = 0.5;
          victimMarker.color.b = 0.3;
          victimDescription.color.r = 0.5;
          victimDescription.color.b = 0.3;
          victimDescription.text = "CANCELLED_" + victimDescription.text;
        }
      }
    }
    else
    {
      if (isTargeted_)
      {
        victimMarker.color.r = 0;
        victimMarker.color.g = 0.7;
        victimMarker.color.b = 0.7;
        victimMarker.color.a = 0.7;
        victimDescription.color.r = 0;
        victimDescription.color.g = 0.7;
        victimDescription.color.b = 0.7;
        victimDescription.color.a = 0.7;
      }
      else
      {
        victimMarker.color.r = 0.94;
        victimMarker.color.g = 0.1255;
        victimMarker.color.b = 0.788;
        victimMarker.color.a = 0.7;
        victimDescription.color.r = 0.94;
        victimDescription.color.g = 0.1255;
        victimDescription.color.b = 0.788;
        victimDescription.color.a = 0.7;
      }
    }

    markers->markers.push_back(victimMarker);
    markers->markers.push_back(victimDescription);
  }

  void Victim::fillGeotiff(const pandora_data_fusion_msgs::GetGeotiffResponsePtr& res) const
  {
    if (valid_)
    {
      pandora_data_fusion_msgs::VictimInfo validVictimInfo;
      validVictimInfo = getVictimInfo();
      validVictimInfo.timeFound = getTimeValidated();
      res->victims.push_back(validVictimInfo);
    }
  }

  void Victim::inspect()
  {
    float probability = 0;
    bool victimVisionFound = false;
    for (int ii = 0; ii < objects_.size(); ++ii) {
      if (objects_[ii]->getType() == VisualVictim::getObjectType())
      {
        setProbability(objects_[ii]->getProbability());
        victimVisionFound = true;
        break;
      }
      else if (objects_[ii]->getType() != Hole::getObjectType())
      {
        probability += objects_[ii]->getProbability();
      }
    }
    if (!victimVisionFound)
    {
      std::vector<std::string> signsOfLife = getSensors(true);
      int numberOfObjects = 2 > signsOfLife.size() ? 2 : signsOfLife.size();
      probability /= numberOfObjects;
      setProbability(probability);
    }
  }

  /**
    * @details Representative Object should be the one in the cluster
    * with the most confidence (least standard deviation-most probability),
    * with its conviction pdf updated by the rest objects in victim.
    * Between the hole and the thermal candidate, hole will be prefered.
    */
  void Victim::setObjects(const ObjectConstPtrVector& objects)
  {
    ROS_DEBUG_STREAM_NAMED("VICTIM_SET_OBJECTS",
        "Setting up victim with " << objects.size() << " objects.");
    objects_.clear();

    findRepresentativeObject<Hole>(objects);
    findRepresentativeObject<Thermal>(objects);
    findRepresentativeObject<VisualVictim>(objects);
    findRepresentativeObject<Motion>(objects);
    findRepresentativeObject<Sound>(objects);
    findRepresentativeObject<Co2>(objects);
    findRepresentativeObject<Hazmat>(objects);
    updateRepresentativeObject();
  }

  std::vector<std::string> Victim::getSensors(bool signsOfLife) const
  {
    std::vector<std::string> sensors;
    for (int ii = 0; ii < objects_.size(); ++ii) {
      if (signsOfLife && objects_[ii]->getType() == Hole::getObjectType())
        continue;
      sensors.push_back(objects_[ii]->getType());
    }
    return sensors;
  }

  pandora_data_fusion_msgs::VictimProbabilities Victim::getProbabilities() const
  {
    pandora_data_fusion_msgs::VictimProbabilities probabilities;
    for (int ii = 0; ii < objects_.size(); ++ii)
    {
      if (objects_[ii]->getType() == VisualVictim::getObjectType())
        probabilities.visualVictim = objects_[ii]->getProbability();
      if (objects_[ii]->getType() == Thermal::getObjectType())
        probabilities.thermal = objects_[ii]->getProbability();
      if (objects_[ii]->getType() == Motion::getObjectType())
        probabilities.motion = objects_[ii]->getProbability();
      if (objects_[ii]->getType() == Co2::getObjectType())
        probabilities.co2 = objects_[ii]->getProbability();
      if (objects_[ii]->getType() == Sound::getObjectType())
        probabilities.sound = objects_[ii]->getProbability();
      if (objects_[ii]->getType() == Hazmat::getObjectType())
        probabilities.hazmat = objects_[ii]->getProbability();
    }

    return probabilities;
  }

  pandora_data_fusion_msgs::VictimInfo Victim::getVictimInfo() const
  {
    pandora_data_fusion_msgs::VictimInfo victimInfo;

    victimInfo.id = getId();
    victimInfo.victimFrameId = getFrameId();
    victimInfo.timeFound = getTimeFound();
    victimInfo.victimPose = getPoseStamped();
    victimInfo.probability = getProbability();
    victimInfo.valid = getValid();

    victimInfo.sensors = getSensors(true);
    victimInfo.probabilities = getProbabilities();
    victimInfo.verified = getVerified();

    return victimInfo;
  }

  /**
    * TODO
    * @details As for now, hole objects are prefered over thermal objects,
    * because it is more likely to verify a victim though vision means
    * rather than thermal sensors. In a future implementation of the robot
    * where the thermal sensor would be more informative and trustworthy this
    * method would have been changed.
    */
  void Victim::updateRepresentativeObject()
  {
    bool isEmpty = objects_.size() == 0;
    selectedObjectIndex_ = -1;
    ROS_DEBUG_COND(isEmpty,
        "[ALERT_HANDLER_VICTIM] Trying to update representative object of an empty victim");
    if (isEmpty)
      return;

    int visualVictimIndex = -1, holeIndex = -1, thermalIndex = -1;
    for (int ii = 0; ii < objects_.size(); ++ii)
    {
      if (objects_[ii]->getType() == VisualVictim::getObjectType())
      {
        visualVictimIndex = ii;
      }
      else if (objects_[ii]->getType() == Hole::getObjectType())
      {
        holeIndex = ii;
      }
      else if (objects_[ii]->getType() == Thermal::getObjectType())
      {
        thermalIndex = ii;
      }
    }

    if (visualVictimIndex != -1)
    {
      selectedObjectIndex_ = visualVictimIndex;
    }
    else if (holeIndex != -1)
    {
      selectedObjectIndex_ = holeIndex;
    }
    else if (thermalIndex != -1)
    {
      selectedObjectIndex_ = thermalIndex;
    }

    if (selectedObjectIndex_ == -1)
    {
      selectedObjectIndex_ = 0;
    }
    setPose(objects_[selectedObjectIndex_]->getPose());
  }

  /**
    * @details Erasing an object will get the appropriate objectDeleted_
    * flag set to true, so that this victim will be ignorant to that type of
    * objects.
    */
  void Victim::eraseObjectAt(int index)
  {
    objects_.erase(objects_.begin() + index);
    updateRepresentativeObject();
  }

  tf::Transform Victim::getRotatedTransform() const
  {
    tf::Transform trans = getTransform();
    tfScalar roll, pitch, yaw;
    trans.getBasis().getRPY(roll, pitch, yaw);
    trans.setRotation(
        tf::createQuaternionFromRPY(roll, pitch, (yaw + PI)));
    return trans;
  }

  tf::Transform Victim::getTransform() const
  {
    tf::Quaternion tfQuaternion(
        pose_.orientation.x, pose_.orientation.y,
        pose_.orientation.z, pose_.orientation.w);
    tf::Vector3 vec(pose_.position.x, pose_.position.y, pose_.position.z);
    return tf::Transform(tfQuaternion, vec);
  }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

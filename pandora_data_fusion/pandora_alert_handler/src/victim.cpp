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

#include "alert_handler/victim.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    Victim::Victim()
    {
      lastVictimId_++;
      id_ = lastVictimId_;
      valid_ = false;
      visited_ = false;
      timeFound_ = ros::Time::now();
      selectedObjectIndex_ = -1;
      probability_ = 0;
    }

    int Victim::lastVictimId_ = 0;

    void Victim::getVisualization(visualization_msgs::MarkerArray* markers) const
    {
      visualization_msgs::Marker victimMarker;
      visualization_msgs::Marker victimDescription;
      victimMarker.header.frame_id = Victim::getGlobalFrame();
      victimDescription.header.frame_id = Victim::getGlobalFrame();
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
      if (visited_)
      {
        victimMarker.color.r = 1;
        victimMarker.color.g = 0;
        victimMarker.color.b = 0;
        victimMarker.color.a = 1;
        victimDescription.color.r = 1;
        victimDescription.color.g = 0;
        victimDescription.color.b = 0;
        victimDescription.color.a = 1;
        if (valid_)
        {
          victimDescription.text = "VALID_" + victimDescription.text + "!!!!";
        }
        else
        {
          victimDescription.text = "REJECTED_" + victimDescription.text;
        }
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

      markers->markers.push_back(victimMarker);
      markers->markers.push_back(victimDescription);
    }

    void Victim::fillGeotiff(pandora_data_fusion_msgs::
        DatafusionGeotiffSrv::Response* res) const
    {
      if (valid_)
      {
        res->victimsx.push_back(pose_.position.x);
        res->victimsy.push_back(pose_.position.y);
      }
    }

    void Victim::inspect()
    {
      float probability = 0;
      bool victimVisionFound = false;
      for (int ii = 0; ii < objects_.size(); ++ii)
      {
        if (objects_[ii]->getType() == Face::getObjectType())
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
        int numberOfObjects = 2 > objects_.size() - 1 ? 2 : objects_.size() - 1;
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
      findRepresentativeObject<Face>(objects);
      findRepresentativeObject<Motion>(objects);
      findRepresentativeObject<Sound>(objects);
      findRepresentativeObject<Co2>(objects);
      updateRepresentativeObject();
    }

    /**
     * @details As for now, hole objects are prefered over thermal objects,
     * because it is more likely to verify a victim though vision means
     * rather than thermal sensors. In a future implementation of the robot
     * where the thermal sensor would be more informative and trustworthy this
     * method would have been changed.
     */
    void Victim::updateRepresentativeObject()
    {
      if (objects_.size() == 0)
      {
        selectedObjectIndex_ = -1;
      }

      int tpaIndex = -1;
      for (int ii = 0; ii < objects_.size(); ++ii)
      {
        if (objects_[ii]->getType() == Hole::getObjectType())
        {
          selectedObjectIndex_ = ii;
        }
        else if (objects_[ii]->getType() == Thermal::getObjectType())
        {
          tpaIndex = ii;
        }
      }

      if (selectedObjectIndex_ == -1)
      {
        selectedObjectIndex_ = tpaIndex;
      }

      if (selectedObjectIndex_ > -1)
      {
        setPose(objects_[selectedObjectIndex_]->getPose());
      }
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


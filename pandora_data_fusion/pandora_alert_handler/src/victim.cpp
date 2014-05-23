// "Copyright [year] <Copyright Owner>" 

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
      selectedObjectIndex_ = -1;
    }

    int Victim::lastVictimId_ = 0;

    PoseStamped Victim::getPoseStamped() const
    {
      PoseStamped victimPose;

      victimPose.pose = pose_;
      if (!visited_)
      {
        victimPose.header.frame_id = "victim_" + boost::to_string(id_);
      }
      else
      {
        if (valid_)
        {
          victimPose.header.frame_id = "VALID_VICTIM" +
            boost::to_string(id_) + "!!!!";
        }
        else
        {
          victimPose.header.frame_id = "deleted_victim_" +
            boost::to_string(id_);
        }
      }

      return victimPose;
    }

    void Victim::getVisualization(visualization_msgs::MarkerArray* markers) const
    {
      //!< fill victim pose
      visualization_msgs::Marker victimMarker;

      victimMarker.header.frame_id = getFrameId();
      victimMarker.header.stamp = ros::Time::now();
      victimMarker.ns = "Victim";
      victimMarker.id = id_;

      victimMarker.pose = pose_;

      victimMarker.type = visualization_msgs::Marker::SPHERE;

      victimMarker.scale.x = 0.1;
      victimMarker.scale.y = 0.1;
      victimMarker.scale.z = 0.1;

      if (visited_)
      {
        victimMarker.color.r = 1;
        victimMarker.color.g = 0;
        victimMarker.color.b = 0;
        victimMarker.color.a = 1;
      }
      else
      {
        victimMarker.color.r = 0.94;
        victimMarker.color.g = 0.1255;
        victimMarker.color.b = 0.788;
        victimMarker.color.a = 0.7;
      }

      markers->markers.push_back(victimMarker);
    }

    void Victim::fillGeotiff(pandora_data_fusion_msgs::
        DatafusionGeotiffSrv::Response* res) const
    {
      if (valid_)
      {
        res->victimsx.push_back( pose_.position.x );
        res->victimsy.push_back( pose_.position.y );
      }
    }

    void Victim::inspect()
    {
      float probability = 0;
      bool victimVisionFound = false;
      for (int ii = 0; ii < objects_.size(); ++ii)
      {
        if(objects_[ii]->getType() == Face::getObjectType())
        {
          setProbability(objects_[ii]->getProbability());
          victimVisionFound = true;
          break;
        }
        else if(objects_[ii]->getType() != Hole::getObjectType())
        {
          probability += objects_[ii]->getProbability();
        }
      }
      if(!victimVisionFound)
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
      ROS_DEBUG_STREAM("Setting up victim with " << objects.size() << " objects.");
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
      if(objects_.size() == 0)
      {
        selectedObjectIndex_ = -1;
      }

      int tpaIndex = -1;
      for(int ii = 0; ii < objects_.size(); ++ii)
      {
        if(objects_[ii]->getType() == Hole::getObjectType())
        {
          selectedObjectIndex_ = ii;
        }
        else if(objects_[ii]->getType() == Thermal::getObjectType())
        {
          tpaIndex = ii;
        }
      }

      if(selectedObjectIndex_ == -1)
      {
        selectedObjectIndex_ = tpaIndex;
      }

      if(selectedObjectIndex_ > -1)
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
          pose_.orientation.z, pose_.orientation.w );
      tf::Vector3 vec(pose_.position.x, pose_.position.y, pose_.position.z);
      return tf::Transform(tfQuaternion, vec);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


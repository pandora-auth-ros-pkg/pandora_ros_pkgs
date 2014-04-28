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
  holeDeleted_ = false;
  tpaDeleted_ = false;
  selectedObjectIndex_ = -1;
}

int Victim::lastVictimId_ = 0;
FilterModelPtr Victim::holeModelPtr_ = FilterModelPtr();
FilterModelPtr Victim::tpaModelPtr_ = FilterModelPtr();

bool Victim::isSameObject(const ObjectConstPtr& object, float distance) const
{  
  ROS_ASSERT(object->getType().compare(type_) == 0);
  
  return Object::isSameObject(object, distance);
}

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

PoseStamped Victim::getApproachPoseStamped() const
{
  PoseStamped approachPoseStamped;
  approachPoseStamped.pose = approachPose_;
  approachPoseStamped.header.frame_id = 
    "app_pose_" + boost::to_string(id_);
  return approachPoseStamped;
}

void Victim::getVisualization(visualization_msgs::MarkerArray* markers) const
{
  //!< fill victim pose
  visualization_msgs::Marker victimMarker;

  victimMarker.header.frame_id = "/world";
  victimMarker.header.stamp = ros::Time::now();
  victimMarker.ns = "Victim";
  victimMarker.id = id_;

  victimMarker.pose = pose_;

  victimMarker.type = visualization_msgs::Marker::SPHERE;

  victimMarker.scale.x = 0.1;
  victimMarker.scale.y = 0.1;
  victimMarker.scale.z = 0.1;

  //!< fill victim approach pose
  visualization_msgs::Marker approachPointMarker;

  approachPointMarker.header.frame_id = "/world";
  approachPointMarker.header.stamp = ros::Time::now();
  approachPointMarker.ns = "ApproachPoint";
  approachPointMarker.id = id_;

  approachPointMarker.pose = approachPose_;

  approachPointMarker.type = visualization_msgs::Marker::ARROW;

  approachPointMarker.scale.x = 0.4;
  approachPointMarker.scale.y = 0.05;
  approachPointMarker.scale.z = 0.05;

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

    approachPointMarker.color.r = 0.94;
    approachPointMarker.color.g = 0.1255;
    approachPointMarker.color.b = 0.788;
    approachPointMarker.color.a = 0.8;

    markers->markers.push_back(approachPointMarker);
  }

  markers->markers.push_back(victimMarker);
}

void Victim::fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const
{
  if (valid_)
  {
    res->victimsx.push_back( pose_.position.x );
    res->victimsy.push_back( pose_.position.y );
  }
}

/**
 * @details Representative Object should be the one in the cluster 
 * with the most confidence (least standard deviation) with its conviction pdf 
 * updated by the rest objects in victim. Between the hole and the tpa 
 * candidate, hole will be prefered.
 */
void Victim::setObjects(const ObjectConstPtrVector& objects,
    float approachDistance) 
{
  ROS_DEBUG_STREAM("Setting up victim with "<<objects.size()<<" objects.");
  objects_.clear();

  if(!holeDeleted_)
  {
    ObjectConstPtrVector::const_iterator holeIt = objects.end();
    float minHoleVariance = 1;

    for ( ObjectConstPtrVector::const_iterator it = objects.begin(); 
        it != objects.end(); it++)
    {
      if (!(*it)->getType().compare(std::string("hole")) && 
          (*it)->getVarianceX() < minHoleVariance)
      {
        minHoleVariance = (*it)->getVarianceX();
        holeIt = it;
      }
    }

    ObjectPtr representativeHole( new Hole );

    if (holeIt != objects.end())
      *representativeHole = *(*holeIt);

    for ( ObjectConstPtrVector::const_iterator it = objects.begin(); 
        it != objects.end(); it++)
    {
      if (!(*it)->getType().compare(std::string("hole")) && it != holeIt)
      {
        representativeHole->update((*it), holeModelPtr_);
      }
    }
  
    if (holeIt != objects.end())
      objects_.push_back(representativeHole);
  }

  if(!tpaDeleted_)
  {
    ObjectConstPtrVector::const_iterator tpaIt = objects.end();
    float minTpaVariance = 1;

    for ( ObjectConstPtrVector::const_iterator it = objects.begin(); 
        it != objects.end(); it++)
    {
      if (!(*it)->getType().compare(std::string("tpa")) && 
          (*it)->getVarianceX() < minTpaVariance)
      {
        minTpaVariance = (*it)->getVarianceX();
        tpaIt = it;
      }
    }

    ObjectPtr representativeTpa( new Tpa );

    if (tpaIt != objects.end())
      *representativeTpa = *(*tpaIt);

    for ( ObjectConstPtrVector::const_iterator it = objects.begin(); 
        it != objects.end(); it++)
    {
      if (!(*it)->getType().compare(std::string("tpa")) && it != tpaIt)
      {
        representativeTpa->update((*it), tpaModelPtr_);
      }
    }
  
    if (tpaIt != objects.end())
      objects_.push_back(representativeTpa);
  }

  updateRepresentativeObject(approachDistance);
}
  
/**
 * @details Should always be called after any change on the objects_.
 */
void Victim::updateRepresentativeObject(float approachDistance)
{  
  selectedObjectIndex_ = findRepresentativeObject();

  if (selectedObjectIndex_> -1)
  {
    updatePose(objects_[selectedObjectIndex_]->getPose(), approachDistance);
  }
}

/**
 * @details As for now, hole objects are prefered over tpa objects,
 * because it is more likely to verify a victim though vision means
 * rather than thermal sensors. In a future implementation of the robot
 * where the thermal sensor would be more informative and trustworthy this
 * method would have been changed.
 */
int Victim::findRepresentativeObject() const
{
  if (objects_.size() == 0)
  {
    return -1;
  }

  for ( int ii = 0 ; ii < objects_.size() ; ++ii)
  {
    if (!objects_[ii]->getType().compare(std::string("hole")))
    {
      return ii;
    }
  }

  return 0;
}

void Victim::updatePose(const Pose& newPose,
    float approachDistance) 
{
  setPose(newPose);
  approachPose_ = calculateApproachPose(approachDistance);
}

void Victim::addSensor(int sensorId)
{
  sensorIds_.insert(sensorId);
} 

/**
 * @details Erasing an object will get the appropriate objectDeleted_
 * flag set to true, so that this victim will be ignorant to that type of
 * objects.
 */
void Victim::eraseObjectAt(int index,
    float approachDistance)
{
  if(objects_[index]->getType() == "hole")
  {
    holeDeleted_ = true;
  }
  else if(objects_[index]->getType() == "tpa")
  {
    tpaDeleted_ = true;
  }
  objects_.erase(objects_.begin() + index);
  updateRepresentativeObject(approachDistance);
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

/**
 * @details Basically returns an approach point that is some length in front of
 * the position of the victim in the direction of its orientation. Also sets
 * its orientation to be the yaw-reversed of the victim's (as it's if we look
 * to the victim).
 */
Pose Victim::calculateApproachPose(float approachDistance) 
    const
{
  tf::Transform transformation = getTransform();

  tf::Vector3 column = transformation.getBasis().getColumn(0);

  Pose approachPose;

  approachPose.position.x = pose_.position.x + approachDistance * column[0];
  approachPose.position.y = pose_.position.y + approachDistance * column[1];
  approachPose.position.z = 0;

  tfScalar roll, pitch, yaw;

  transformation.getBasis().getRPY(roll, pitch, yaw);

  approachPose.orientation =
    tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, (yaw + PI));

  return approachPose;
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


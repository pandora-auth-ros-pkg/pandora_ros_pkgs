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
*   Christos Zalidis <zalidis@gmail.com>
*   Triantafyllos Afouras <afourast@gmail.com>
*********************************************************************/

#include "alert_handler/victim_list.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

VictimList::VictimList()
{
  currentVictimIt_ = objects_.end();
  victimsRequestedAndGiven_ = false;
  currentVictimDied_ = false;
}

bool VictimList::contains(const VictimConstPtr& victim) const
{
  for (const_iterator it = this->begin(); it != this->end(); ++it)
  {
    if ((*it)->isSameObject(victim, DIST_THRESHOLD))
    {
      return true;
    }
  }
  return false;
}

/**
 * @details A victimToUpdate from victim list is selected (this will be the 
 * current if we are tracking one). Its info is updated by copying the given 
 * victim's objects and the rest victims that are thought to be the same are 
 * deleted from victim list. The fsm is informed if necessary.
 */
void VictimList::updateObjects(const VictimConstPtr& victim, 
    const IteratorList& iteratorList)
{                                   
  ROS_ASSERT(iteratorList.size() > 0);

  const_iterator_vers_ref victimToUpdate = *(iteratorList.begin());

  if (currentVictimIt_ != objects_.end() )
  {
    for ( IteratorList::const_iterator it = iteratorList.begin() ;
      it != iteratorList.end() ; ++it)
    {
      if ((*currentVictimIt_)->getId() == (*(*it))->getId())
      {
        victimToUpdate = *it;
        break;
      }
    }
  }

  (*victimToUpdate)->setObjects(victim->getObjects(), APPROACH_DIST);
  
  for ( IteratorList::const_iterator it = iteratorList.begin();
      it != iteratorList.end(); ++it)
  {
    if ( *(it) == victimToUpdate )
    {
      continue;
    }
    objects_.erase( *(it) );
  }
}

/**
 * @details Sets various thresholds and params for VictimList
 */
void VictimList::setParams(float distanceThreshold, 
    float approachDistance, float victimUpdate)
{    
  ObjectList<Victim>::setParams(0, distanceThreshold);
  APPROACH_DIST = approachDistance;
  VICTIM_UPDATE = victimUpdate;  
}

/**
 * @details It is set that if we do not currently track a victim,
 * the iterator is the one pointing at the end of the victim list
 * (where there is no victim - yet)
 */
bool VictimList::isVictimBeingTracked() const
{
  return currentVictimIt_ != objects_.end();
}

/**
 * @details Assuming that a victim is being tracked,
 * this method returns that victim.
 */
const VictimPtr& VictimList::getCurrentVictim() const
{
  ROS_ASSERT(currentVictimIt_ != objects_.end());
  return *currentVictimIt_;
}

/**
 * @details Keeps track of the victims' indices in the sequence they are 
 * returned so that when setCurrentVictimIndex is called, we can still find
 * the correct index by its victimId_.
 */
void VictimList::getVictimsMsg(
    std::vector< data_fusion_communications::VictimInfoMsg>* victimMsgVector)
{
  victimMsgVector->clear();

  victimIndicesMap_.clear();

  ros::Time now = ros::Time::now();

  int ii = 0;
  for (const_iterator it = this->begin(); it != this->end(); ++it)
  {
    victimIndicesMap_[ii++] = (*it)->getId();

    data_fusion_communications::VictimInfoMsg victimInfo;

    victimInfo.victimPose.header.stamp = now;
    victimInfo.victimPose.header.frame_id = (*it)->getFrameId();
    victimInfo.victimPose.pose = (*it)->getApproachPose();
    victimInfo.probability = (*it)->getProbability();

    victimMsgVector->push_back(victimInfo);
  }

  victimsRequestedAndGiven_ = true;
}

/**
 * @details This method should always be called after getVictimsMsg() so if 
 * that is not the case the assertion should fire. Also, index = -1 means 
 * that no victim was chosen from victimMsgVector.
 */
bool VictimList::setCurrentVictim(int index)
{
  ROS_ASSERT(victimsRequestedAndGiven_);
  victimsRequestedAndGiven_ = false;

  if(index == -1)
  {
    currentVictimIt_ = objects_.end();
    return true;
  }

  int victimId = victimIndicesMap_[index];

  for (iterator it = objects_.begin(); it != objects_.end(); ++it)
  {
    if  ((*it)->getId() == victimId)
    {
      currentVictimIt_ = it;
      currentApproachPose_ = (*currentVictimIt_)->getApproachPose();
      currentVictimDied_ = false;
      return true;
    }
  }
  return false;
}

/**
 * @details If there is a victim being tracked returns its transform
 * with reversed yaw. Else, returns false.
 */
bool VictimList::getCurrentVictimTransform(tf::Transform* Transform) const
{
  if (currentVictimIt_ == objects_.end())
  {
    return false;
  }
  *Transform = (*currentVictimIt_)->getRotatedTransform();
  return true;
}

/**
 * @details Sets victim's info according to the message from Victim Fusion.
 */
bool VictimList::updateCurrentVictimSensorsAndProb(
    const data_fusion_communications::VictimVerificationMsg& msg)
{    
  if (currentVictimIt_ == objects_.end())
  {
    return false;
  }
    
  // maybe use a probability function
  (*currentVictimIt_)->setProbability(msg.probability);
  for (int i = 0; i < msg.sensorIds.size(); i++)
  {
    (*currentVictimIt_)->addSensor(msg.sensorIds[i]);
  }

  return true;
}

/**
 * @details Assuming that a victim is being tracked, that victim is erased
 * from victim list. Next currentVictim iterator points to the end of the
 * list.
 */
bool VictimList::deleteCurrentVictim()
{
  ROS_ASSERT(currentVictimIt_ != objects_.end());
  objects_.erase(currentVictimIt_);
  currentVictimIt_ = objects_.end();
  return true;
}

/**
 * @details If the object is valid then the current victim is erased from the
 * list and returned. If it is not valid, it is erased from the victim's 
 * objects. If after this erasal the victim is empty, then it is erased and 
 * returned.
 */
VictimPtr VictimList::validateCurrentObject(bool objectValid)
{
  ROS_ASSERT(currentVictimIt_ != objects_.end());

  VictimPtr currentVictim;
  
  if (objectValid)
  {
    currentVictim = *currentVictimIt_;
    currentVictim->setValid(true);
    currentVictim->setVisited(true);
    objects_.erase(currentVictimIt_);
    currentVictimIt_ = objects_.end();
  }
  else
  {
    (*currentVictimIt_)->eraseObjectAt(
      (*currentVictimIt_)->getSelectedObjectIndex(), APPROACH_DIST);
    if ((*currentVictimIt_)->getObjects().empty() || 
        (*currentVictimIt_)->getObjects().at(0)->getType() == "tpa")  // to be removed
    {
      currentVictim = *currentVictimIt_;
      currentVictim->setValid(false);
      currentVictim->setVisited(true);
      objects_.erase(currentVictimIt_);
      currentVictimIt_ = objects_.end();
    } 
  }
  
  return currentVictim;
}

/**
 * @details ~Add As You Are~ - Nirvana
 */
void VictimList::addUnchanged(const VictimPtr& victim)
{
  objects_.push_back(victim);
}

void VictimList::clear()
{
  objects_.clear();
  currentVictimIt_ = objects_.end();  
}

/**
 * @details Has the victim being tracked changed state?
 * If it has died or its approach point has changed much,
 * it is necessary to inform.
 */
bool VictimList::currentVictimUpdated()
{
  if (currentVictimIt_ == objects_.end())
  {
    if (currentVictimDied_)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  if (Utils::distanceBetweenPoints2D(
        (*currentVictimIt_)->getApproachPose().position,
          currentApproachPose_.position) > VICTIM_UPDATE)
  {
    return true;
  }
  return false;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


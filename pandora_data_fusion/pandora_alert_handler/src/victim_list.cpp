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
#include <vector>

VictimList::VictimList(int counterThreshold, float distanceThreshold, 
      float approachDistance, float victimUpdate) :
  ObjectList<Victim>(counterThreshold, distanceThreshold) {
    currentVictimIt_ = objects_.end();
    victimsRequestedAndGiven_ = false;
    currentVictimDied_ = false;
    APPROACH_DIST = approachDistance;
    VICTIM_UPDATE = victimUpdate;
  }

/**
@details Also returns a vector containing the indices of the victims in the
  vector that match the given one
**/
bool VictimList::contains(const VictimPtr& victim) const {
  for (const_iterator it = objects_.begin(); it != objects_.end(); ++it) {
    if ((*it)->isSameObject(victim, DIST_THRESHOLD)) {
      return true;
    }
  }
  return false;
}


/**
@details A victim from given the indices of the unvisited list is selected 
  (this will be the current if we track one). It's info is updated by 
    copying the given victim object's one and the rest are deleted from the 
      unvisited list. The fsm is informed if necessary     
**/
void VictimList::updateObject(const VictimPtr& victim,
                                 const IteratorList& iteratorList) {
                                   
  ROS_ASSERT(iteratorList.size() > 0);

  const_iterator victimToUpdate = *(iteratorList.begin());

  if (currentVictimIt_ != objects_.end() ) {
    for ( IteratorList::const_iterator it = iteratorList.begin() ;
      it != iteratorList.end() ; ++it) {
      if ((*currentVictimIt_)->getId() == (*(*it))->getId()) {
        victimToUpdate = *it;
        break;
      }
    }
  }

  (*victimToUpdate)->setObjects(victim->getObjects(), APPROACH_DIST);
  
  for ( IteratorList::const_iterator it = iteratorList.begin();
      it != iteratorList.end() ; ++it) {
    if ( *(it) == victimToUpdate ) {
      continue;
    }
    objects_.erase( *(it) );
  }

}

/**
@details 
**/
void VictimList::setParams(int counterThreshold, float distanceThreshold, 
  float approachDistance, float victimUpdate) {
    
    ObjectList<Victim>::setParams(counterThreshold, distanceThreshold);
    APPROACH_DIST = approachDistance;
    VICTIM_UPDATE = victimUpdate;
    
  }

/**
@details 
**/
bool VictimList::isVictimBeingTracked() const {
  return currentVictimIt_ != objects_.end();
}

/**
@details 
**/
const VictimPtr& VictimList::getCurrentVictim() const {
  ROS_ASSERT(currentVictimIt_ != objects_.end());
  return *currentVictimIt_;
}

/**
@details Keeps track of the victims indices in the sequence they are 
  returned so that when setCurrentVictimIndex is called, we can still find
  the correct index
**/
void VictimList::getVictimsMsg(
  std::vector< data_fusion_communications::VictimInfoMsg>* victimMsgVector) {

  victimMsgVector->clear();

  victimIndicesMap_.clear();

  ros::Time now = ros::Time::now();

  int ii = 0;
  for (const_iterator it = objects_.begin(); it != objects_.end(); ++it) {

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
@details This method should always be called after getVictimsMsg() so if that
  is not the case the assertion should fire.
  Victim id = -1 means that no victim was chosen so
  if a victim is tracked it is unset. This behavior may very possibly change 
**/
bool VictimList::setCurrentVictim(int index) {

  ROS_ASSERT(victimsRequestedAndGiven_);
  victimsRequestedAndGiven_ = false;

  if(index == -1) {
    currentVictimIt_ = objects_.end();
    return true;
  }

  int victimId = victimIndicesMap_[index];

  for (iterator it = objects_.begin(); it != objects_.end(); ++it) {
    if  ((*it)->getId() == victimId) {
      currentVictimIt_ = it;
      currentApproachPose_ = (*currentVictimIt_)->getApproachPose();
      currentVictimDied_ = false;
      return true;
    }
  }
  return false;
  
}

/**
@details 
**/
bool VictimList::getCurrentVictimTransform(tf::Transform* Transform) const {
  if (currentVictimIt_ == objects_.end()) {
    return false;
  }
  *Transform =  (*currentVictimIt_)->getRotatedTransform();
  return true;
}

/**
@details 
**/
bool VictimList::updateCurrentVictimSensorsAndProb(
  const data_fusion_communications::VictimVerificationMsg& msg) {
    
  if (currentVictimIt_ == objects_.end()) {
    return false;
  }
    
  // maybe use a probability function
  (*currentVictimIt_)->setProbability(msg.probability);
  for (int i = 0; i < msg.sensorIds.size(); i++) {
    (*currentVictimIt_)->addSensor(msg.sensorIds[i]);
  }

}

/**
@details 
**/
bool VictimList::deleteCurrentVictim() {
  ROS_ASSERT(currentVictimIt_ != objects_.end());
  objects_.erase(currentVictimIt_);
  currentVictimIt_ = objects_.end();
}

/**
@details If the objcect is valid then the current victim is erased from the
 list and returned. If it is not valid, it is erased from the victim's objects.
 If after this erasal the victim is empty, then it is erased and returned
**/
VictimPtr VictimList::validateCurrentObject(bool objectValid) {

  ROS_ASSERT(currentVictimIt_ != objects_.end());

  VictimPtr currentVictim;
  
  if (objectValid) {
    currentVictim = *currentVictimIt_;
    currentVictim->setValid(true);
    currentVictim->setVisited(true);
    objects_.erase(currentVictimIt_);
  } else {
    (*currentVictimIt_)->eraseObjectAt(
      currentVictim->getSelectedObjectIndex(), APPROACH_DIST);
    if (currentVictim->getObjects().empty()) {
      currentVictim = *currentVictimIt_;
      currentVictim->setValid(false);
      currentVictim->setVisited(true);
      objects_.erase(currentVictimIt_);
    } 
  }
  currentVictimIt_ = objects_.end();
  
  return currentVictim;
}

/**
@details 
**/
void VictimList::addUnchanged(const VictimPtr& victim) {
  objects_.push_back(victim);
}

void VictimList::clear() {
  objects_.clear();
  currentVictimIt_ = objects_.end();  
}

/**
@details 
**/
bool VictimList::currentVictimUpdated() {
  if (currentVictimIt_ == objects_.end()) {
    if (currentVictimDied_) {
      return true;
    } else {
      return false;
    }
  }
  if (Utils::distanceBetweenPoints2D(
        (*currentVictimIt_)->getApproachPose().position,
          currentApproachPose_.position) > VICTIM_UPDATE) {
    return true;
  }
}

/**
@details If a victim is erased during sanity check, we should inform it,
  that's why currentVictimDied_ the is used. See currentVictimUpdated()
**/
void VictimList::sanityCheck(const ObjectPtrVector& allObjects) {
  
  iterator it = objects_.begin();
    
  while (it != objects_.end()) {
    (*it)->sanityCheck(allObjects, DIST_THRESHOLD, APPROACH_DIST);
    if((*it)->getObjects().empty()) {
      if (it == currentVictimIt_) {
        currentVictimIt_ =  objects_.end();
        currentVictimDied_ = true;
      }
      objects_.erase(it++);
    } else {
      ++it;
    }
  }
  
}



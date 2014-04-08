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

#ifndef ALERT_HANDLER_VICTIM_LIST_H
#define ALERT_HANDLER_VICTIM_LIST_H

#include <list>
#include <vector>
#include <map>

#include "data_fusion_communications/VictimInfoMsg.h"
#include "data_fusion_communications/VictimVerificationMsg.h"

#include "alert_handler/objects.h"
#include "alert_handler/object_list.h"
#include "alert_handler/utils.h"
#include "alert_handler/victim.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class VictimList : public ObjectList<Victim>
{  
 public:
  
  /**
  @brief Constructor
  @param counterThreshold [float] Initialization value for counterThreshold
  @param distanceThreshold [float] Initialization value for distanceThreshold
  @param approachDistance [float] Initialization value for approachDistance
  **/ 
  VictimList(int counterThreshold = 1, float distanceThreshold = 0.5, 
    float approachDistance = 0.5, float victimUpdate = 0.5 );

  /**
  @brief Checks if the given victim is already in the list
  @param victim [VictimPtr] The victim whose existance we need to check
  @return bool True if victim exists, false otherwise
  **/
  bool contains(const VictimConstPtr& victim) const;

  /**
  @brief Checks if a victim is currently tracked or not
  @return bool True if a victim is currently tracked, false otherwise
  **/
  bool isVictimBeingTracked() const;

  /**
  @brief Returns the currently tracked victim
  @return const VictimPtr& A const& to the victim
  **/
  const VictimPtr& getCurrentVictim() const; 

    /**
  @brief Returns a vector containing a VictimInfoMsg for each unvisited victim
  @param victimMsgVector 
    [std::vector< data_fusion_communications::VictimInfoMsg >*] 
      The output vector
  @return void
  **/
  void getVictimsMsg(
    std::vector< data_fusion_communications::VictimInfoMsg >* victimMsgVector);

  /**
  @brief Sets the victim index to a specific victim
  @param index [int] the index of the previeously returned victims selected 
  @return bool Whether the victim with the given id was found or not. 
    Also true for -1 
  **/
  bool setCurrentVictim(int victimId);

  /**
  @brief Get the current victim pose as a stamped transform
  @param stampedTranform [tf::StampedTransform*] The output param
  @return bool -1 if no victim is tracked
  **/
  bool getCurrentVictimTransform(tf::Transform* Transform) const;

  /**
  @brief Updates current victim with sensors and fusion probability
  @param msg [data_fusion_communications::VictimVerificationMsg&] : 
    The victim verirification message
  @return bool -1 if no victim is tracked
  **/
  bool updateCurrentVictimSensorsAndProb(
    const data_fusion_communications::VictimVerificationMsg& msg);

  /**
  @brief Deletes the current VictimPtr  
  @return bool -1 if no victim is tracked
  **/
  bool deleteCurrentVictim();

  /**
  @brief Validates or not currentObject. If object is erased then 
    currentVictim is returned
  @param objectValid [bool] If current object is valid or not
  @return The current object pointer 
  **/
  VictimPtr validateCurrentObject(bool objectValid);

  /**
  @brief Adds the given victim to the list as is, no checks and updates
  @param victim [VictimPtr] The victim to be added
  @return void
  **/
  void addUnchanged(const VictimPtr& victim);

  /**
  @brief Checks if the approach pose of current victim has significally changed
    sinse start of tracking
  @return bool True if approach pose changed a significally, false otherwise
  **/
  bool currentVictimUpdated(); 

  /**
  @brief Performs a check to see if all the objects that comprise each
    victim still exist
  @param allObjects [const ObjectPtrVector&] All the legit objects
  @return void
  **/
  void sanityCheck(const ObjectConstPtrVectorPtr& allObjects);

  /**
  @override
  **/
  void clear();

  /**
  @overload
  **/
  void setParams(int counterThreshold, float distanceThreshold, 
    float approachDistance, float victimUpdate);

 protected: 
  
  /**
  @override
  **/
  void updateObject(const VictimPtr& victim,
                                 const IteratorList& iteratorList);
  
 protected:
 
  //!< An iterator pointing to the currently tracked victim 
  iterator currentVictimIt_;
  //!< The pose of the currently tracked victim when tracking was last updated 
  geometry_msgs::Pose currentApproachPose_;
  //!< A map containing correspondence of victims returned indices with ids
  std::map<int, int> victimIndicesMap_;
  //!< True if the victims were requested and given, false otherwise
  bool victimsRequestedAndGiven_;
  //!< True if the victims were requested and given, false otherwise
  bool currentVictimDied_;
  
  //!< The distance of the approach pose from the wall
  float APPROACH_DIST;
  //!< The approach pose distance threshold for informing fsm of change 
  float VICTIM_UPDATE;

 private:

  friend class VictimListTest;
 
};

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_VICTIM_LIST_H

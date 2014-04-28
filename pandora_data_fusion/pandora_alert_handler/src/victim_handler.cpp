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

#include "alert_handler/victim_handler.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

/**
 * @details 
 */
VictimHandler::VictimHandler(const HoleListConstPtr& holeListPtr,
                             const TpaListConstPtr& tpaListPtr) :
  holePtrListPtr_(holeListPtr),
  tpaPtrListPtr_(tpaListPtr)
{
  Victim::setHoleModel(holePtrListPtr_->getFilterModel());
  Victim::setTpaModel(tpaPtrListPtr_->getFilterModel());

  clusterer_.reset( new VictimClusterer(0.2, 0.5) );

  validVictimsCounter_ = 0;

  std::string param; 
  
  if (nh_.getParam("published_topic_names/victim_found", param))
  {
    victimFoundPublisher_ =  
      nh_.advertise<data_fusion_communications::VictimFoundMsg>(param, 1);
  }
  else
  {
    ROS_FATAL("victim_found topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("published_topic_names/victim_update", param))
  {
    victimUpdatePublisher_ = nh_.advertise<std_msgs::Empty>(param, 1);
  }
  else
  {
    ROS_FATAL("victim_update topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("published_topic_names/victim_verified", param))
  {
    victimVerifiedPublisher_ =
      nh_.advertise<data_fusion_communications::VictimToFsmMsg>(param, 1);
  }
  else
  {
    ROS_FATAL("victim_verified topic name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("published_topic_names/valid_victims_counter", param))
  {
    validVictimsPublisher_ =  
      nh_.advertise<std_msgs::Int32>(param, 1);
  }
  else
  {
    ROS_FATAL("valid_victims_counter topic name param not found");
    ROS_BREAK();
  }
}

/**
 * @details Clusters the existing Objects into victims and then updates
 * the list with the unvisited victims with it.
 */
void VictimHandler::notify()
{  
  ObjectConstPtrVectorPtr allObjects = getAllLegitObjects();  

  VictimPtrVector newVictimVector = clusterer_->createVictimList(allObjects);

  for (int ii = 0; ii < newVictimVector.size(); ii++)
  {
    if ( victimsVisitedList_.contains( newVictimVector[ii] ) )
    {
      continue;
    }

    victimsToGoList_.add(newVictimVector[ii]);

    if ( !victimsToGoList_.isVictimBeingTracked() )
    {
      publishVictimFoundMsg();
    }
    else if (victimsToGoList_.currentVictimUpdated())
    {
      publishVictimUpdatedMsg();
    }
  }
}

/**
 * @details Collects from hole and tpa list, all these objects
 * that are thought to be legitimate and are to be grouped to 
 * Victim objects.
 */
ObjectConstPtrVectorPtr VictimHandler::getAllLegitObjects()
{
  ObjectConstPtrVectorPtr result(new ObjectConstPtrVector);

  ObjectList<Hole>::const_iterator holeIt;

  for ( holeIt = holePtrListPtr_->begin();
        holeIt != holePtrListPtr_->end() ; ++holeIt )
  {
    if ( (*holeIt) -> getLegit())
    {
      result->push_back(HoleConstPtr(*holeIt));
    }
  }

  ObjectList<Tpa>::const_iterator tpaIt;

  for ( tpaIt = tpaPtrListPtr_->begin();
        tpaIt != tpaPtrListPtr_->end() ; ++tpaIt )
  {
    if ( (*tpaIt) -> getLegit())
    {
      result->push_back(TpaConstPtr(*tpaIt));
    }
  }

  return result;
}

/**
 * @details Delegate to victimList
 */
void VictimHandler::getVictimsMsg (
  std::vector< data_fusion_communications::VictimInfoMsg>* victimMsgVector)
{  
  victimsToGoList_.getVictimsMsg(victimMsgVector); 
}

/**
 * @details Delegate to victimList
 */
void VictimHandler::setCurrentVictimIndex(int index)
{
    // race condition !! If victim is
    // deleted before navigation sends back selected id.
    // Hopefully this will be quite uncommon.
    // should fix if ASSERTION ever fails
    bool victimWasSet = victimsToGoList_.setCurrentVictim(index);
    ROS_ASSERT(victimWasSet);
}

/**
 * @details Delegate to victimList and add Stamp
 */
bool VictimHandler::getCurrentVictimTransform(
    tf::StampedTransform* stampedTranform)
{
  tf::Transform trans;
  bool victimTracked = victimsToGoList_.getCurrentVictimTransform(&trans);
  if (victimTracked)
  {
    *stampedTranform =  tf::StampedTransform(
        trans, ros::Time::now(), "world", "current_victim" );
    return true;
  } 
  return false;
}

/**
 * @details Delegate to victimList and publish verification msg if probability
 * exceeds threshold
 */
void VictimHandler::handleVictimVerification(
    const data_fusion_communications::VictimVerificationMsg& msg)
{
  bool victimTracked = victimsToGoList_.updateCurrentVictimSensorsAndProb(msg);
  
  if(!victimTracked)
  {
    ROS_ERROR("[VICTIM_HANDLER %d] VictimVerificationMsg was"
              "called when no victim is tracked ", __LINE__);
    return;
  }

  if(msg.probability > VICTIM_VERIFICATION_PROB)
  {
    publishVictimToFsmMsg(victimsToGoList_.getCurrentVictim());
  }
}


/**
 * @details Delegate to victimList
 */
void VictimHandler::deleteCurrentVictim()
{
  bool victimTracked = victimsToGoList_.deleteCurrentVictim();

  if(!victimTracked)
  {
    ROS_ERROR("[VICTIM_HANDLER %d] deleteCurrentVictim was"
              "called when no victim is tracked ", __LINE__);
  }
}

/**
 * @details Delegate to victimList
 */
void VictimHandler::validateCurrentHole(bool objectValid)
{
  VictimPtr currentVictim = victimsToGoList_.validateCurrentObject(objectValid);
  
  if(currentVictim.get())
  {
    if(currentVictim->getValid())
    {
      std_msgs::Int32 updateValidVictims;
      updateValidVictims.data = ++validVictimsCounter_;
      validVictimsPublisher_.publish(updateValidVictims);
    }
    victimsVisitedList_.addUnchanged(currentVictim);
  }
}

////////////////////////////////////////////////////////////////////////////////

void VictimHandler::getVictimsPosesStamped(PoseStampedVector* victimsToGo, 
    PoseStampedVector* victimsVisited, PoseStampedVector* approachPoses)
{
  victimsToGoList_.getObjectsPosesStamped(victimsToGo);
  victimsVisitedList_.getObjectsPosesStamped(victimsVisited);
  for(VictimList::const_iterator it = victimsToGoList_.begin();
      it != victimsToGoList_.end(); ++it)
  {
    approachPoses->push_back((*it)->getApproachPoseStamped());
  }
}

/**
 * @details 
 */
void VictimHandler::fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res)
{
  victimsVisitedList_.fillGeotiff(res);
}

/**
 * @details 
 */
void VictimHandler::getVisualization(
    visualization_msgs::MarkerArray* victimsVisitedMarkers,
    visualization_msgs::MarkerArray* victimsToGoMarkers)
{
  victimsVisitedList_.getVisualization(victimsVisitedMarkers);
  victimsToGoList_.getVisualization(victimsToGoMarkers);
}

/**
 * @details 
 */
void  VictimHandler::publishVictimFoundMsg()
{
  ROS_INFO_NAMED("victim_handler",
                 "[VICTIM_HANDLER %d] New victim found ", __LINE__);
  data_fusion_communications::VictimFoundMsg victimMsg;
  victimMsg.victimNotificationType = victimMsg.TYPE_CAMERA;
  victimFoundPublisher_.publish(victimMsg);
}

/**
 * @details 
 */
void VictimHandler::publishVictimUpdatedMsg() 
{
  std_msgs::Empty msg;

  victimUpdatePublisher_.publish(msg);
}

/**
 * @details 
 */
void VictimHandler::publishVictimToFsmMsg(const VictimPtr& victim)
{
  data_fusion_communications::VictimToFsmMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/world";
  msg.x = victim->getPose().position.x;
  msg.y = victim->getPose().position.y;
  msg.probability = victim->getProbability();
  for (std::set<int>::iterator it = victim->getSensorIds().begin();
       it != victim->getSensorIds().end(); ++it)
  {
    msg.sensors.push_back(sensorIdToString(*it));
  }
  victimVerifiedPublisher_.publish(msg);
}

/**
 * @details 
 */
std::string VictimHandler::sensorIdToString(int sensorId)
{
  switch (sensorId)
  {
    case data_fusion_communications::VictimVerificationMsg::FACE:
      return "Face";
    case data_fusion_communications::VictimVerificationMsg::MOTION:
      return "Motion";
    case data_fusion_communications::VictimVerificationMsg::MLX:
      return "Mlx";
    case data_fusion_communications::VictimVerificationMsg::CO2:
      return "CO2";
  }
  ROS_ERROR("[VICTIM_HANDLER %d] sensorIdToString was called"
            "with invalid sensor id", __LINE__);
  ROS_BREAK();
  return "";
}


/**
 * @details 
 */
void VictimHandler::updateParams(float clusterRadius, float sameVictimRadius,
                                 float approachDist, float victimUpdate,
                                 float verificationProbability)
{
  VICTIM_VERIFICATION_PROB = verificationProbability;
  clusterer_->updateParams(clusterRadius, approachDist);
  victimsToGoList_.setParams(sameVictimRadius, approachDist, victimUpdate);
  victimsVisitedList_.setParams(sameVictimRadius, approachDist, victimUpdate);
}

/**
 * @details 
 */
void VictimHandler::flush()
{
  victimsToGoList_.clear();
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


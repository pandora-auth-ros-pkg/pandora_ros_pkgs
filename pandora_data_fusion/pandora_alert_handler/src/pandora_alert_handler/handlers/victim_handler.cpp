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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <vector>
#include <string>

#include "pandora_alert_handler/handlers/victim_handler.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  VictimHandler::VictimHandler(
      const ros::NodeHandlePtr& nh, const std::string& globalFrame,
      VictimListPtr victimsToGoList, VictimListPtr victimsVisitedList) :
    victimsToGoList_(victimsToGoList),
    victimsVisitedList_(victimsVisitedList)
  {
    std::string param;

    nh->param<std::string>("object_names/victim", param, std::string("victim"));
    Victim::setObjectType(param);
    Victim::is3D = true;

    clusterer_.reset( new VictimClusterer(globalFrame, 0.2) );

    if (nh->getParam("published_topic_names/victim_probabilities", param))
    {
      probabilitiesPublisher_ = nh->
        advertise<pandora_data_fusion_msgs::VictimProbabilities>(param, 2);
    }
    else
    {
      ROS_FATAL("victim probabilities topic name param not found");
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
      // Escape if it is already visited
      if (victimsVisitedList_->contains(newVictimVector[ii]))
        continue;
      // Escape if it has only a hazmat
      std::vector<std::string> sensors = newVictimVector[ii]->getSensors(false);
      if (sensors.size() == 1)
      {
        if (sensors[0] == Hazmat::getObjectType())
          continue;
        if (sensors[0] == Sound::getObjectType())
        {
          if (boost::dynamic_pointer_cast<Sound const>(newVictimVector[ii]->getObjects().at(0))->
              isWordsEmpty())
            continue;
        }
      }
      bool victimIsNew = victimsToGoList_->add(newVictimVector[ii]);
      if (victimIsNew)
        ROS_INFO("[ALERT_HANDLER] New victim found!!!");
    }
  }

  /**
    * @details Use notify() to create victims, then delegate inspection of
    * victims to VictimList.
    */
  void VictimHandler::inspect()
  {
    notify();
    victimsToGoList_->inspect();

    if (victimsToGoList_->size() != 0)
    {
      VictimConstPtr currentVictim;
      if (targetedVictim_.get() == NULL) {
        currentVictim = *(victimsToGoList_->begin());
        ros::Time oldestVictim = currentVictim->getTimeFound();
        for (VictimList::const_iterator it = ++victimsToGoList_->begin();
            it != victimsToGoList_->end(); it++)
        {
          if ((*it)->getTimeFound() < oldestVictim)
          {
            currentVictim = *it;
            oldestVictim = currentVictim->getTimeFound();
          }
        }
      }
      else
      {
        currentVictim = targetedVictim_;
      }

      pandora_data_fusion_msgs::VictimProbabilities probabilities;
      probabilities = currentVictim->getProbabilities();

      probabilitiesPublisher_.publish(probabilities);
    }
  }

  /**
    * @details Collects from object lists, all these objects
    * that are thought to be legitimate and are to be grouped to
    * Victim objects. Delegates to ObjectList.
    */
  ObjectConstPtrVectorPtr VictimHandler::getAllLegitObjects()
  {
    ObjectConstPtrVectorPtr result( new ObjectConstPtrVector );

    Hole::getList()->getAllLegitObjects(result);
    Thermal::getList()->getAllLegitObjects(result);
    VisualVictim::getList()->getAllLegitObjects(result);
    Motion::getList()->getAllLegitObjects(result);
    Sound::getList()->getAllLegitObjects(result);
    Co2::getList()->getAllLegitObjects(result);
    Hazmat::getList()->getAllLegitObjects(result);

    return result;
  }

  bool VictimHandler::targetVictim(int victimId)
  {
    targetedVictim_ = victimsToGoList_->targetVictim(victimId);
    if (targetedVictim_.get())
      return true;
    return false;
  }

  /**
    * @details Delegate to victimList
    */
  bool VictimHandler::deleteVictim(int victimId)
  {
    VictimPtr deletedVictim( new Victim );
    bool deleted = victimsToGoList_->deleteVictim(victimId, deletedVictim);
    if (deleted)
    {
      Hole::getList()->removeInRangeOfObject(deletedVictim, CLUSTER_RADIUS);
      Thermal::getList()->removeInRangeOfObject(deletedVictim, CLUSTER_RADIUS);
      VisualVictim::getList()->removeInRangeOfObject(deletedVictim, CLUSTER_RADIUS);
      Motion::getList()->removeInRangeOfObject(deletedVictim, CLUSTER_RADIUS);
      Sound::getList()->removeInRangeOfObject(deletedVictim, CLUSTER_RADIUS);
      Co2::getList()->removeInRangeOfObject(deletedVictim, CLUSTER_RADIUS);
      if (targetedVictim_->getId() == victimId) {
        targetedVictim_.reset();
      }
    }
    return deleted;
  }

  /**
    * @details Delegate to victimList
    */
  bool VictimHandler::validateVictim(int victimId, bool victimVerified,
      bool victimValid)
  {
    VictimPtr currentVictim = victimsToGoList_->validateVictim(victimId,
        victimVerified, victimValid);

    if (currentVictim.get())
    {
      victimsVisitedList_->addUnchanged(currentVictim);
      if (targetedVictim_->getId() == victimId) {
        targetedVictim_.reset();
      }
      return true;
    }
    return false;
  }

  ////////////////////////////////////////////////////////////////////////////////

  /**
    * @details Delegate to victimList
    */
  void VictimHandler::getVictimsInfo(
      pandora_data_fusion_msgs::WorldModel* worldModelMsg)
  {
    victimsToGoList_->getVictimsInfo(&(worldModelMsg->victims));
    victimsVisitedList_->getVictimsInfo(&(worldModelMsg->visitedVictims));
  }

  void VictimHandler::getVictimsPosesStamped(PoseStampedVector* victimsToGo,
      PoseStampedVector* victimsVisited)
  {
    victimsToGoList_->getObjectsPosesStamped(victimsToGo);
    victimsVisitedList_->getObjectsPosesStamped(victimsVisited);
  }

  /**
    * @details
    */
  void VictimHandler::fillGeotiff(
      const pandora_data_fusion_msgs::GetGeotiffResponsePtr& res)
  {
    victimsVisitedList_->fillGeotiff(res);
  }

  /**
    * @details
    */
  void VictimHandler::getVisualization(
      visualization_msgs::MarkerArray* victimsVisitedMarkers,
      visualization_msgs::MarkerArray* victimsToGoMarkers)
  {
    victimsVisitedList_->getVisualization(victimsVisitedMarkers);
    victimsToGoList_->getVisualization(victimsToGoMarkers);
  }

  bool VictimHandler::getVictimProbabilities(int victimId,
      const pandora_data_fusion_msgs::GetVictimProbabilitiesResponsePtr& rs)
  {
    bool found = false;
    for (VictimList::const_iterator it = victimsToGoList_->begin();
         it != victimsToGoList_->end(); ++it) {
      if ((*it)->getId() == victimId) {
        found = true;
        rs->sensors = (*it)->getSensors(true);
        rs->probabilities = (*it)->getProbabilities();
      }
    }
    return found;
  }

  /**
    * @details
    */
  void VictimHandler::updateParams(float clusterRadius, float sameVictimRadius)
  {
    CLUSTER_RADIUS = clusterRadius;
    clusterer_->updateParams(clusterRadius);
    Victim::setDistanceThres(sameVictimRadius);
  }

  /**
    * @details
    */
  void VictimHandler::flush()
  {
    victimsToGoList_->clear();
  }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

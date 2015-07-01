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

#include "pandora_alert_handler/handlers/victim_clusterer.h"

namespace pandora_data_fusion
{

using pandora_data_fusion_utils::Utils;

namespace pandora_alert_handler
{

  VictimClusterer::VictimClusterer(
      const std::string& globalFrame, float clusterRadius) :
    globalFrame_(globalFrame)
  {
    CLUSTER_RADIUS = clusterRadius;
  }

  /**
    * @details In order to create Victim objects, two things are being used.
    * First, a simple centroid clusterer groups all given objects into vectors
    * of objects. Next, a Victim object is created by each vector of grouped
    * objects by finding their representative object and setting it in Victim's
    * characteristic objects.
    */
  VictimPtrVector VictimClusterer::createVictimList(
      const ObjectConstPtrVectorPtr& allObjects)
  {
    ObjectConstPtrVectorVector groupedObjects = groupObjects(allObjects);

    VictimPtrVector newVictimVector;

    for (int ii = 0; ii < groupedObjects.size(); ++ii) {
      VictimPtr newVictim( new Victim );
      newVictim->setGlobalFrame(globalFrame_);
      newVictim->setObjects(groupedObjects[ii]);
      newVictimVector.push_back(newVictim);
    }

    return newVictimVector;
  }

  /**
    * @details Very simple clusterer that uses a centroid and a euclidean distance
    * metric with threshold to decide whether an object should be added
    * to the group or not.
    */
  ObjectConstPtrVectorVector
    VictimClusterer::groupObjects(const ObjectConstPtrVectorPtr& allObjects)
    {
      ObjectConstPtrVectorVector groupedObjects;

      for (int objectIt = 0 ; objectIt < allObjects->size() ; ++objectIt) {
        ObjectConstPtr currentObj = allObjects->at(objectIt);

        bool isAdded = false;

        for (int ii = 0; ii < groupedObjects.size(); ++ii) {
          geometry_msgs::Point groupCenterPoint =
            findGroupCenterPoint(groupedObjects[ii]);

          double distance = 0;
          if (currentObj->getType() != Sound::getObjectType() &&
              currentObj->getType() != Co2::getObjectType())
          {
            distance =
              Utils::distanceBetweenPoints3D(currentObj->
                  getPose().position, groupCenterPoint);
          }
          else
          {
            distance =
              Utils::distanceBetweenPoints2D(currentObj->
                  getPose().position, groupCenterPoint);
          }

          if (distance < CLUSTER_RADIUS)
          {
            groupedObjects[ii].push_back(currentObj);
            isAdded = true;
            if (currentObj->getType() != Sound::getObjectType() &&
                currentObj->getType() != Co2::getObjectType())
              break;
          }
        }

        if (!isAdded)
        {
          ObjectConstPtrVector newVect;
          newVect.push_back(currentObj);
          groupedObjects.push_back(newVect);
          isAdded = false;
        }
      }

      return groupedObjects;
    }

  /**
    * @details Given a group of objects which contain 3D position coordinates,
    * this function returns the centroid of the group.
    */
  geometry_msgs::Point VictimClusterer::findGroupCenterPoint(
      const ObjectConstPtrVector& objects)
  {
    geometry_msgs::Point centerPoint;

    for (ObjectConstPtrVector::const_iterator it = objects.begin();
        it != objects.end(); ++it) {
      centerPoint.x += (*it)->getPose().position.x;
      centerPoint.y += (*it)->getPose().position.y;
      centerPoint.z += (*it)->getPose().position.z;
    }

    centerPoint.x /= objects.size();
    centerPoint.y /= objects.size();
    centerPoint.z /= objects.size();

    return centerPoint;
  }

  /**
    * @details Updates the parameters tha are used in clustering (distance
    * threshold).
    */
  void VictimClusterer::updateParams(float clusterRadius)
  {
    CLUSTER_RADIUS = clusterRadius;
  }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


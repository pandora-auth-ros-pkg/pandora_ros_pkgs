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

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_VICTIM_CLUSTERER_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_VICTIM_CLUSTERER_H_

#include <vector>

#include "ros/ros.h"

#include "alert_handler/victim.h"

/**
  @class VictimClusterer
  @brief Controller that keeps track of victims 
**/ 
class VictimClusterer {

 public:
 
  /**
  @brief Constructor
  **/
  VictimClusterer(float clusterRadius, float approachDist);

  /**
  @brief Creates a new victim vector from groups of Objects
  @param groupedObjects [ObjectPtrVectorVector] The vector containing the
    groups of Objects
  @return VictimPtrVector The resulting victim vector 
  **/
  VictimPtrVector createVictimList(ObjectPtrVector allObjects);

  /**
  @brief Updates the victim handler's parameters
  @param clusterRadius [float] The new cluster radius
  @return void
  **/
  void updateParams(float clusterRadius, float approachDist);

 private:

  /**
  @brief Clusters the existing Objects into victims
  @return ObjectPtrVectorVector A vector containing
    the resulting groups as vectors of Objects
  **/
  ObjectPtrVectorVector groupObjects(ObjectPtrVector allObjects);

  /**
  @brief Finds and returns the centroid of a group of Objects
  @param objects [ObjectPtrVector] The group of Objects
  @return geometry_msgs::Point The centroid
  **/
  geometry_msgs::Point findGroupCenterPoint(ObjectPtrVector objects);


 private:
  
  //!< The radius used for clustering
  float CLUSTER_RADIUS;
  //!< The distance of the approach pose from the wall
  float APPROACH_DIST;

};

typedef boost::shared_ptr<VictimClusterer> VictimClustererPtr;

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_VICTIM_CLUSTERER_H_

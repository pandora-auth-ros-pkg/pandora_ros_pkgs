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

#ifndef PANDORA_ALERT_HANDLER_HANDLERS_VICTIM_CLUSTERER_H
#define PANDORA_ALERT_HANDLER_HANDLERS_VICTIM_CLUSTERER_H

#include <vector>
#include <boost/utility.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>

#include "pandora_alert_handler/objects/victim.h"
#include "pandora_data_fusion_utils/defines.h"
#include "pandora_data_fusion_utils/utils.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  /**
    * @class VictimClusterer
    * @brief Controller that keeps track of victims
    */
  class VictimClusterer : private boost::noncopyable
  {
   public:
    /**
      * @brief Constructor
      */
    VictimClusterer(const std::string& globalFrame, float clusterRadius);

    /**
      * @brief Creates a new victim vector from groups of Objects
      * @param groupedObjects [ObjectPtrVectorVector] The vector containing the
      * groups of Objects
      * @return VictimPtrVector The resulting victim vector
      */
    VictimPtrVector createVictimList(
        const ObjectConstPtrVectorPtr& allObjects);

    /**
      * @brief Updates the victim handler's parameters
      * @param clusterRadius [float] The new cluster radius
      * @return void
      */
    void updateParams(float clusterRadius);

   private:
    /**
      * @brief Clusters the existing Objects into victims
      * @return ObjectPtrVectorVector A vector containing
      * the resulting groups as vectors of Objects
      */
    ObjectConstPtrVectorVector groupObjects(const ObjectConstPtrVectorPtr& allObjects);

    /**
      * @brief Finds and returns the centroid of a group of Objects
      * @param objects [ObjectPtrVector] The group of Objects
      * @return geometry_msgs::Point The centroid
      */
    geometry_msgs::Point findGroupCenterPoint(const ObjectConstPtrVector& objects);

   private:
    //!< Map origin to which victim poses are refering
    std::string globalFrame_;
    //!< The radius used for clustering
    float CLUSTER_RADIUS;

   private:
    friend class VictimClustererTest;
  };

  typedef boost::scoped_ptr<VictimClusterer> VictimClustererPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_HANDLERS_VICTIM_CLUSTERER_H

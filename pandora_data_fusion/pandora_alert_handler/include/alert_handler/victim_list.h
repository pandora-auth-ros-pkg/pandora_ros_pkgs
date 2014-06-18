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

#ifndef ALERT_HANDLER_VICTIM_LIST_H
#define ALERT_HANDLER_VICTIM_LIST_H

#include <list>
#include <vector>
#include <map>

#include "pandora_data_fusion_msgs/WorldModelMsg.h"
#include "pandora_data_fusion_msgs/VictimInfoMsg.h"

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
         * @brief Constructor
         * @param counterThreshold [float] Initialization value for counterThreshold
         * @param distanceThreshold [float] Initialization value for distanceThreshold
         * @param approachDistance [float] Initialization value for approachDistance
         */ 
        VictimList();

        /**
         * @brief Checks if the given victim is already in the list
         * @param victim [VictimConstPtr const&] The victim whose existance
         * we need to check
         * @return bool True if victim exists, false otherwise
         */
        bool contains(const VictimConstPtr& victim) const;

        /**
         * @brief Inspects the objects contained in each victim and calculates 
         * an associated probability to the victim.
         * @return void
         */
        void inspect();

        /**
         * @brief Returns a vector containing a VictimInfoMsg for each unvisited victim
         * @param victimsMsg [pandora_data_fusion_msgs::WorldModelMsg*] The output vector
         * @return void
         */
        void getVictimsInfo(
            std::vector<pandora_data_fusion_msgs::VictimInfoMsg>* victimsMsg);

        /**
         * @brief Deletes VictimPtr with the corresponding victimId
         * @param victimId [int] id that will be used to search for the victim
         * @param deletedVictim [VictimPtr const&] victim whose objects are
         * to be deleted
         * @return bool true, if deleted, false, if not found.
         */
        bool deleteVictim(int victimId, const VictimPtr& deletedVictim);

        /**
         * @brief Validates victim with victimId.
         * @param victimId [int] current victim's unique id
         * @param victimValid [bool] If current victim is valid or not
         * @return VictimPtr pointer to current victim
         */
        VictimPtr validateVictim(int victimId, bool objectValid);

        /**
         * @brief Adds the given victim to the list as is, no checks and updates
         * @param victim [VictimPtr const&] The victim to be added
         * @return void
         */
        void addUnchanged(const VictimPtr& victim);

        /**
         * @override
         */
        void clear();

      protected:

        /**
         * @override
         */
        void updateObjects(const ConstPtr& victim,
            const IteratorList& iteratorList);

      private:

        friend class VictimListTest;
    };

    typedef boost::shared_ptr<VictimList> VictimListPtr;
    typedef boost::shared_ptr<const VictimList> VictimListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_VICTIM_LIST_H

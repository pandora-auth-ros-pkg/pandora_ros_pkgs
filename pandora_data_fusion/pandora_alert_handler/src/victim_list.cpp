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

#include "alert_handler/victim_list.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    VictimList::VictimList() {}

    bool VictimList::contains(const VictimConstPtr& victim) const
    {
      for (const_iterator it = this->begin(); it != this->end(); ++it)
      {
        if ((*it)->isSameObject(victim))
        {
          return true;
        }
      }
      return false;
    }

    void VictimList::inspect()
    {
      for (iterator it = objects_.begin(); it != objects_.end(); ++it)
      {
        (*it)->inspect();
      }
    }

    /**
     * @details A victimToUpdate from victim list is selected (this will be the 
     * oldest among the ones from iteratorList). Its info is updated by copying 
     * the given victim's objects and the rest victims that are thought to be the 
     * same are deleted from victim list.
     */
    void VictimList::updateObjects(const ConstPtr& victim,
        const IteratorList& iteratorList)
    {
      ROS_ASSERT(iteratorList.size() > 0);

      iterator victimToUpdate = *(iteratorList.begin());
      ros::Time oldestVictim = (*victimToUpdate)->getTimeFound();

      for(IteratorList::const_iterator it = ++iteratorList.begin();
          it != iteratorList.end() ; ++it)
      {
        if((*(*it))->getTimeFound() < oldestVictim)
        {
          oldestVictim = (*(*it))->getTimeFound();
          victimToUpdate = *it;
        }
      }

      (*victimToUpdate)->setObjects(victim->getObjects());

      for(IteratorList::const_iterator it = iteratorList.begin();
          it != iteratorList.end(); ++it)
      {
        if((*it) == victimToUpdate)
        {
          continue;
        }
        objects_.erase(*(it));
      }
    }

    /**
     * @details Fills victimsMsg with information about current victim list.
     * Information given consists of a unique id, victim's pose stamped, probability
     * and sensors.
     */
    void VictimList::getVictimsInfo(
        std::vector<pandora_data_fusion_msgs::VictimInfoMsg>* victimsMsg)
    {
      victimsMsg->clear();

      ros::Time now = ros::Time::now();

      for (const_iterator it = this->begin(); it != this->end(); ++it)
      {
        pandora_data_fusion_msgs::VictimInfoMsg victimInfo;

        victimInfo.id = (*it)->getId();
        victimInfo.victimPose.header.stamp = now;
        victimInfo.victimPose.header.frame_id = (*it)->getFrameId();
        victimInfo.victimPose.pose = (*it)->getPose();
        victimInfo.probability = (*it)->getProbability();
        for (ObjectConstPtrVector::const_iterator iter = (*it)->getObjects().begin();
            iter != (*it)->getObjects().end(); ++iter)
        {
          if((*iter)->getType() != Hole::getObjectType())
          {
            victimInfo.sensors.push_back((*iter)->getType());
          }
        }
        victimInfo.valid = (*it)->getValid();

        victimsMsg->push_back(victimInfo);
      }
    }

    /**
     * @details By Agent's order that victim is erased
     * from victim list. Deleted victim is returned to search and delete
     * its associated objects from their respective lists.
     */
    bool VictimList::deleteVictim(int victimId, const VictimPtr& deletedVictim)
    {
      for(VictimList::iterator it = objects_.begin();
          it != objects_.end(); ++it)
      {
        if((*it)->getId() == victimId)
        {
          deletedVictim->setPose((*it)->getPose());
          objects_.erase(it);
          return true;
        }
      }
      return false;
    }

    /**
     * @details Sets the appropriate victim visited. 
     * Also, sets its validation variable according to agent's order. 
     * Next this victim is deleted from victim list and returned.
     */
    VictimPtr VictimList::validateVictim(int victimId, bool victimValid)
    {
      VictimPtr currentVictim;

      for(VictimList::iterator it = objects_.begin();
          it != objects_.end(); ++it)
      {
        if((*it)->getId() == victimId)
        {
          currentVictim = *it;
          currentVictim->setValid(victimValid);
          currentVictim->setVisited(true);
          objects_.erase(it);
          break;
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
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


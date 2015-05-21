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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef ALERT_HANDLER_VICTIM_H
#define ALERT_HANDLER_VICTIM_H

#include <vector>

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Victim
     * @brief Class to describe a victim in the world
     * @details Inherits from Object
     */
    class Victim : public Object<Victim>
    {
      public:
        //!< Type definitions
        typedef boost::shared_ptr<Victim> Ptr;
        typedef boost::shared_ptr<Victim const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;

      public:
        /**
         * @brief Default constructor
         */
        Victim();

        /**
         * @brief Setter of the objects associated with the Victim. Selects and keeps
         * as many objects as the different types of objects comprising this victim.
         * @param objects [ObjectConstPtrVector const&]
         * Group of objects considered a victim
         * @param approachDistance [float]
         * Approach point's desired distance away from victim
         * @return void
         */
        void setObjects(const ObjectConstPtrVector& objects);

        /**
         * @brief Inspects this victim's objects in order to verify it.
         * Sets its probability.
         * @return void
         */
        void inspect();

        /**
         * @brief Erases from victim's associated objects
         * @param index [int] Index in victim's vector of objects
         * @param approachDistance [float]
         * Approach point's desired distance away from victim
         * @return void
         */
        void eraseObjectAt(int index);

        /**
         * @override
         * @brief Getter for geotiff information about the victim
         * @param res [data_fusion...::GeotiffSrv::Response*]
         * @return void
         */
        virtual void fillGeotiff(pandora_data_fusion_msgs::
            GeotiffSrv::Response* res) const;

        /**
         * @override
         * @brief Getter for visualization markers of victim
         * @param markers [visualization_msgs::MarkerArray*] Pointer to marker array
         * @return void
         */
        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;

        /**
         * @brief Getter for member valid_
         * @return bool valid_
         */
        bool getValid() const
        {
          return valid_;
        }

        /**
         * @brief Getter for member visited_
         * @return bool visited_
         */
        bool getVisited() const
        {
          return visited_;
        }

        /**
         * @brief Getter for member selectedObjectIndex_
         * @return bool selectedObjectIndex_
         */
        int getSelectedObjectIndex() const
        {
          return selectedObjectIndex_;
        }

        /**
         * @brief Getter for member objects_
         * @return std::set<int>& objects_
         */
        const ObjectConstPtrVector& getObjects() const
        {
          return objects_;
        }

        /**
         * @brief Getter for victim's transform from /world (yaw-reversed)
         * @return tf::Transform
         */
        tf::Transform getRotatedTransform() const;

        /**
         * @brief Setter for member valid_
         * @param valid [bool] The new valid_ value
         * @return void
         */
        void setValid(bool valid)
        {
          valid_ = valid;
        }

        /**
         * @brief Setter for member visited_
         * @param visited [bool] The new visited_ value
         * @return void
         */
        void setVisited(bool visited)
        {
          visited_ = visited;
        }

      private:
        /**
         * @brief Updates the representative object and consequently the pose
         * @param approachDistance [float] The disired distance from the wall
         * @details Should be always called after any change on the objects_
         * @return void
         */
        void updateRepresentativeObject();

        /**
         * @brief Selects the object to represent victim's pose out of victim's
         * grouped object
         * @param approachDistance [float] distance of approch position from victim
         * @return int index of selected object
         */
        template <class ObjectType>
          void findRepresentativeObject(const ObjectConstPtrVector& objects);

        /**
         * @brief Gets the transform from /world to the victim
         * @return tf::Transform
         */
        tf::Transform getTransform() const;

      protected:
        //!< The validity of the victim
        bool valid_;
        //!< True if the victim was visited false otherwise
        bool visited_;

        //!< Index pointing to representative object
        int selectedObjectIndex_;

        //!< Victim's characteristic group of objects
        ObjectConstPtrVector objects_;

      private:
        static int lastVictimId_;  //!< The last in line victim ID

      private:
        friend class VictimTest;
    };

    typedef Victim::Ptr VictimPtr;
    typedef Victim::ConstPtr VictimConstPtr;
    typedef Victim::PtrVector VictimPtrVector;
    typedef Victim::PtrVectorPtr VictimPtrVectorPtr;

    template <class ObjectType>
    void Victim::findRepresentativeObject(const ObjectConstPtrVector& objects)
    {
      ObjectConstPtrVector::const_iterator objectIt = objects.end();
      float maxObjectProbability = 0;

      for (ObjectConstPtrVector::const_iterator it = objects.begin();
          it != objects.end(); it++)
      {
        if ((*it)->getType() == ObjectType::getObjectType() &&
            (*it)->getProbability() > maxObjectProbability)
        {
          maxObjectProbability = (*it)->getProbability();
          objectIt = it;
        }
      }

      typename ObjectType::Ptr representativeObject(new ObjectType);

      if (objectIt != objects.end())
        *representativeObject = *(boost::dynamic_pointer_cast<const ObjectType>(*objectIt));

      for (ObjectConstPtrVector::const_iterator it = objects.begin();
          it != objects.end(); it++)
      {
        if ((*it)->getType() == ObjectType::getObjectType() && it != objectIt)
        {
          representativeObject->update((*it));
        }
      }

      if (objectIt != objects.end())
        objects_.push_back(representativeObject);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_VICTIM_H

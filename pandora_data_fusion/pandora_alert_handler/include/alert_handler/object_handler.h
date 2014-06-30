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

#ifndef ALERT_HANDLER_OBJECT_HANDLER_H
#define ALERT_HANDLER_OBJECT_HANDLER_H

#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

#include <std_msgs/Int32.h>

#include "pandora_data_fusion_msgs/QrNotificationMsg.h"

#include "alert_handler/objects.h"
#include "alert_handler/object_list.h"
#include "alert_handler/victim_list.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    //!< Type Definitions
    typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;

    /**
     * @brief Class which is responsible for keeping or ignoring
     * alerts, as well as placing them in their appropriate lists.
     */
    class ObjectHandler : private boost::noncopyable
    {
      public:
        /**
         * @brief constructor
         * @param nh [NodeHandlePtr const&] Alert Handler's node to register publishers
         * @param victimsToGoList [VictimListConstPtr const&]
         * list with victims to go - to be used in alert filtering
         * @param victimsVisited [VictimListConstPtr const&]
         * list with victims visited - to be used in alert filtering
         */
        ObjectHandler(
            const NodeHandlePtr& nh,
            const VictimListConstPtr& victimsToGoList,
            const VictimListConstPtr& victimsVisited);

        void handleHoles(const HolePtrVectorPtr& newHoles,
            const tf::Transform& transform);
        void handleQrs(const QrPtrVectorPtr& newQrs);
        /**
         * @brief methods that handle alerts after their creation.
         * They try to keep those of interest and ignore the rest.
         * @param objectsPtr [ObjectType::PtrVectorPtr const&] vector with alerts
         * @return void
         */
        template <class ObjectType>
          void handleObjects(
              const typename ObjectType::PtrVectorPtr& objectsPtr);

        /**
         * @brief parameter updating from dynamic reconfiguration
         * @param sensor_range [float] sensor's range defines maximum distance
         * one can have from the alert.
         * @param victim_cluster_radius [float] defines if an alert can be associated
         * with a victim in question [identification mode]
         * @return void
         */
        void updateParams(float sensor_range, float victim_cluster_radius);

      private:
        /**
         * @brief Alert filtering for holes.
         * @param holesPtr [HolePtrVectorPtr const&] vector with newly
         * created hole alerts
         * @param cameraTransform [tf::Transform const&] camera transform gives
         * current location
         * @return void
         */
        void keepValidHoles(const HolePtrVectorPtr& holesPtr,
            const tf::Transform& cameraTransform);
        template <class ObjectType>
          void keepValidVerificationObjects(
              const typename ObjectType::PtrVectorPtr& objectsPtr);

      private:
        ros::Publisher qrPublisher_;
        ros::Publisher scorePublisher_;

        VictimListConstPtr victimsToGoList_;
        VictimListConstPtr victimsVisitedList_;

        int roboCupScore_;

        float SENSOR_RANGE;
        float VICTIM_CLUSTER_RADIUS;
    };

    template <class ObjectType>
      void ObjectHandler::handleObjects(
          const typename ObjectType::PtrVectorPtr& newObjects)
      {
        if (ObjectType::getObjectType() != Thermal::getObjectType() &&
            ObjectType::getObjectType() != Hazmat::getObjectType() &&
            ObjectType::getObjectType() != Landoltc::getObjectType() &&
            ObjectType::getObjectType() != DataMatrix::getObjectType())
        {
          keepValidVerificationObjects<ObjectType>(newObjects);
        }
        for (int ii = 0; ii < newObjects->size(); ++ii)
        {
          if (ObjectType::getList()->add(newObjects->at(ii)))
          {
            std_msgs::Int32 updateScoreMsg;
            roboCupScore_ += ObjectType::getObjectScore();
            updateScoreMsg.data = roboCupScore_;
            scorePublisher_.publish(updateScoreMsg);
          }
        }
      }

    template <class ObjectType>
      void ObjectHandler::keepValidVerificationObjects(
          const typename ObjectType::PtrVectorPtr& objectsPtr)
      {
        typename ObjectType::PtrVector::iterator iter = objectsPtr->begin();

        while (iter != objectsPtr->end())
        {
          bool valid = false;
          if (ObjectType::getObjectType() == Sound::getObjectType() ||
              ObjectType::getObjectType() == Co2::getObjectType())
          {
            valid = victimsToGoList_->isObjectPoseInList(
                (*iter), VICTIM_CLUSTER_RADIUS, false);
          }
          else
            valid = victimsToGoList_->isObjectPoseInList(
                (*iter), VICTIM_CLUSTER_RADIUS);
          // valid = valid ||
          //   victimsVisitedList_->isObjectPoseInList((*iter), VICTIM_CLUSTER_RADIUS);
          if (!valid)
          {
            ROS_DEBUG_NAMED("OBJECT_HANDLER",
                "[OBJECT_HANDLER %d] Deleting not valid object...", __LINE__);
            iter = objectsPtr->erase(iter);
          }
          else
          {
            ++iter;
          }
        }
      }

    typedef boost::scoped_ptr< ObjectHandler >  ObjectHandlerPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_HANDLER_H

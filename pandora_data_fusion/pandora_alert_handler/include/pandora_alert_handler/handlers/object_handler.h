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

#ifndef PANDORA_ALERT_HANDLER_HANDLERS_OBJECT_HANDLER_H
#define PANDORA_ALERT_HANDLER_HANDLERS_OBJECT_HANDLER_H

#include <boost/utility.hpp>
#include <boost/scoped_ptr.hpp>

#include <std_msgs/Int32.h>

#include "pandora_data_fusion_msgs/QrInfo.h"
#include "pandora_data_fusion_msgs/ObstacleInfo.h"

#include "pandora_alert_handler/objects/objects.h"
#include "pandora_alert_handler/object_lists/object_list.h"
#include "pandora_alert_handler/object_lists/obstacle_list.h"
#include "pandora_alert_handler/object_lists/victim_list.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

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
        const ros::NodeHandlePtr& nh,
        const VictimListConstPtr& victimsToGoList,
        const VictimListConstPtr& victimsVisited);

    void handleHoles(const HolePtrVectorPtr& newHoles,
        const tf::Transform& transform);
    /**
      * @brief methods that handle alerts after their creation.
      * They try to keep those of interest and ignore the rest.
      * @param objectsPtr [ObjectType::PtrVectorPtr const&] vector with alerts
      * @return void
      */
    template <class ObjectType>
    void handleObjects(
        const typename ObjectType::PtrVectorPtr& objectsPtr,
        const tf::Transform& transform);

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
      * @param objectsPtr [ObjectType::PtrVectorPtr const&] vector with newly
      * created object alerts
      * @param cameraTransform [tf::Transform const&] camera transform gives
      * current location
      * @return void
      */
    template <class ObjectType>
    void keepValidObjects(const typename ObjectType::PtrVectorPtr& objectsPtr,
        const tf::Transform& cameraTransform);
    template <class ObjectType>
    void deleteObjectsOnSoftObstacles(const typename ObjectType::PtrVectorPtr& objectsPtr);
    template <class ObjectType>
    void keepValidVerificationObjects(
        const typename ObjectType::PtrVectorPtr& objectsPtr);

   private:
    ros::Publisher qrPublisher_;
    ros::Publisher scorePublisher_;
    ros::Publisher obstaclePublisher_;

    VictimListConstPtr victimsToGoList_;
    VictimListConstPtr victimsVisitedList_;

    int roboCupScore_;

    float SENSOR_RANGE;
    float VICTIM_CLUSTER_RADIUS;
  };

  template <class ObjectType>
  void ObjectHandler::keepValidObjects(
      const typename ObjectType::PtrVectorPtr& objectsPtr,
      const tf::Transform& transform)
  {
    tf::Vector3 origin = transform.getOrigin();
    geometry_msgs::Point framePosition = pandora_data_fusion_utils::Utils::
      vector3ToPoint(origin);

    typename ObjectType::PtrVector::iterator iter = objectsPtr->begin();

    while (iter != objectsPtr->end())
    {
      bool invalid = !pandora_data_fusion_utils::Utils::
        arePointsInRange((*iter)->getPose().position, framePosition,
                         ObjectType::is3D, SENSOR_RANGE);

      if (invalid)
      {
        ROS_INFO_NAMED("PANDORA_ALERT_HANDLER",
            "[OBJECT_HANDLER %d] Deleting not valid object...", __LINE__);
        ROS_INFO_NAMED("PANDORA_ALERT_HANDLER",
            "[OBJECT_HANDLER %d] SENSOR_RANGE = %f", __LINE__, SENSOR_RANGE);
        iter = objectsPtr->erase(iter);
      }
      else
      {
        ++iter;
      }
    }
  }

  template <class ObjectType>
  void ObjectHandler::deleteObjectsOnSoftObstacles(
      const typename ObjectType::PtrVectorPtr& objectsPtr)
  {
    ROS_DEBUG("[OBJECT_HANDLER %d] Deleting %ss on soft obstacles", __LINE__, ObjectType::getObjectType().c_str());
    typename ObjectType::PtrVector::iterator iter = objectsPtr->begin();

    while (iter != objectsPtr->end()) {
      bool onSoft = false;
      onSoft = boost::dynamic_pointer_cast<ObstacleList>(Obstacle::getList())
        ->isObjectPoseOnSoftObstacles(*iter);
      if (onSoft)
      {
        ROS_WARN_NAMED("OBJECT_HANDLER",
            "[OBJECT_HANDLER %d] Deleting object on a soft obstacle...", __LINE__);
        iter = objectsPtr->erase(iter);
      }
      else
      {
        ++iter;
      }
    }
  }

  /**
    * @details Sound and CO2 pois give us spatial information in a 2d
    * surface on their sensors' tf frame's plane. We should not search for them
    * in a sphere but in a cylinder.
    */
  template <class ObjectType>
  void ObjectHandler::keepValidVerificationObjects(
      const typename ObjectType::PtrVectorPtr& objectsPtr)
  {
    typename ObjectType::PtrVector::iterator iter = objectsPtr->begin();

    while (iter != objectsPtr->end()) {
      bool valid = false;
      valid = victimsToGoList_->isObjectPoseInList(
          (*iter), VICTIM_CLUSTER_RADIUS);
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

  /**
    * @details keepValidVerificationObjects should not be called for
    * symbol pois (qr, hazmat, landoltc, datamatrix) as well as thermal
    */
  template <class ObjectType>
  void ObjectHandler::handleObjects(
      const typename ObjectType::PtrVectorPtr& newObjects,
      const tf::Transform& transform)
  {
    if (ObjectType::getObjectType() != VisualVictim::getObjectType() &&
        ObjectType::getObjectType() != Hazmat::getObjectType() &&
        ObjectType::getObjectType() != Landoltc::getObjectType() &&
        ObjectType::getObjectType() != DataMatrix::getObjectType()) {
      keepValidVerificationObjects<ObjectType>(newObjects);
    }
    deleteObjectsOnSoftObstacles<ObjectType>(newObjects);
    for (int ii = 0; ii < newObjects->size(); ++ii) {
      if (ObjectType::getList()->add(newObjects->at(ii)))
      {
        std_msgs::Int32 updateScoreMsg;
        roboCupScore_ += ObjectType::getObjectScore();
        updateScoreMsg.data = roboCupScore_;
        scorePublisher_.publish(updateScoreMsg);
      }
    }
  }

  template <>
  void ObjectHandler::handleObjects<Thermal>(
      const typename Thermal::PtrVectorPtr& newObjects,
      const tf::Transform& transform)
  {
    keepValidObjects<Thermal>(newObjects, transform);
    deleteObjectsOnSoftObstacles<Thermal>(newObjects);
    for (int ii = 0; ii < newObjects->size(); ++ii) {
      if (Thermal::getList()->add(newObjects->at(ii)))
      {
        std_msgs::Int32 updateScoreMsg;
        roboCupScore_ += Thermal::getObjectScore();
        updateScoreMsg.data = roboCupScore_;
        scorePublisher_.publish(updateScoreMsg);
      }
    }
  }

  template <>
  void ObjectHandler::handleObjects<Qr>(
      const typename Qr::PtrVectorPtr& newQrs,
      const tf::Transform& transform)
  {
    deleteObjectsOnSoftObstacles<Qr>(newQrs);
    for (int ii = 0; ii < newQrs->size(); ++ii) {
      if (Qr::getList()->add(newQrs->at(ii)))
      {
        pandora_data_fusion_msgs::QrInfo qrInfo;
        qrInfo = newQrs->at(ii)->getQrInfo();
        qrPublisher_.publish(qrInfo);
        std_msgs::Int32 updateScoreMsg;
        roboCupScore_ += Qr::getObjectScore();
        updateScoreMsg.data = roboCupScore_;
        scorePublisher_.publish(updateScoreMsg);
      }
    }
  }

  template <>
  void ObjectHandler::handleObjects<Obstacle>(
      const typename Obstacle::PtrVectorPtr& newObstacles,
      const tf::Transform& transform)
  {
    for (int ii = 0; ii < newObstacles->size(); ++ii) {
      ObstaclePtr obstacleToSend;
      bool obstacleToSendFound = true;
      if (Obstacle::getList()->add(newObstacles->at(ii)))
      {
        ROS_DEBUG("[ObjectHandler %d] Found new obstacle!", __LINE__);
        obstacleToSend = newObstacles->at(ii);
      }
      else
      {
        ROS_DEBUG("[ObjectHandler %d] Fetching old obstacle", __LINE__);
        // Fetch object from list
        typename Obstacle::List::IteratorList obstacleListIteratorList;
        obstacleToSendFound = Obstacle::getList()->isAnExistingObject(
            newObstacles->at(ii), &obstacleListIteratorList);
        if (obstacleToSendFound)
        {
          ROS_WARN_COND(obstacleListIteratorList.size() != 1,
              "[ObjectHandler %d] New obstacle matched with more than one old ones", __LINE__);
          obstacleToSend = *(*obstacleListIteratorList.begin());
        }
      }
      if (obstacleToSendFound)
      {
        // Create and send info message to navigation
        pandora_data_fusion_msgs::ObstacleInfo obstacleInfo;
        obstacleInfo = obstacleToSend->getObstacleInfo();
        // Publish order for obstacle costmap
        obstaclePublisher_.publish(obstacleInfo);
      }
    }
  }

  typedef boost::scoped_ptr< ObjectHandler >  ObjectHandlerPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_HANDLERS_OBJECT_HANDLER_H

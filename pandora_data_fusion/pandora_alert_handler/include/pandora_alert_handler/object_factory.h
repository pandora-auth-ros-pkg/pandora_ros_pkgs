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

#ifndef PANDORA_ALERT_HANDLER_OBJECT_FACTORY_H
#define PANDORA_ALERT_HANDLER_OBJECT_FACTORY_H

#include <boost/utility.hpp>
#include <boost/scoped_ptr.hpp>
#include <string>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>

#include "pandora_vision_msgs/ObstacleAlert.h"

#include "pose_finder/pose_finder.h"
#include "pandora_alert_handler/objects/object_interface/base_object.h"
#include "pandora_alert_handler/objects/objects.h"
#include "pandora_data_fusion_utils/utils.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  class ObjectFactory : private boost::noncopyable
  {
   public:
    ObjectFactory(const pose_finder::PoseFinderPtr& poseFinderPtr,
                  const std::string& globalFrame);

    template <class ObjectType>
    typename ObjectType::PtrVectorPtr makeObjects(
        const typename ObjectType::AlertVector& msg);

    tf::Transform getCurrentTransform() const
    {
      return currentTransform_;
    }

    void dynamicReconfigForward(float occupiedCellThres,
        float highThres, float lowThres,
        float orientationCircle, double soft_obstacle_width);

   private:
    /**
      * @brief Sets this Object up according to the info from the Alert.
      * @param objectPtr [const ObjectPtr&] Pointer to Object
      * variable to be filled.
      * @param msg [const ..._communications::...Msg&]
      * Incoming ros message containing info.
      * @return void
      */
    template <class ObjectType>
    void setUpObject(
        const typename ObjectType::Ptr& objectPtr,
        const typename ObjectType::Alert& msg,
        const ros::Time& timeFound,
        const tf::Transform& transform);

   private:
    tf::Transform currentTransform_;

    std::string globalFrame_;
    pose_finder::PoseFinderPtr poseFinderPtr_;

    double SOFT_OBSTACLE_WIDTH;
  };

  template <class ObjectType>
  void ObjectFactory::setUpObject(
      const typename ObjectType::Ptr& objectPtr,
      const typename ObjectType::Alert& msg,
      const ros::Time& timeFound,
      const tf::Transform& transform)
  {
    objectPtr->setPose(poseFinderPtr_->findAlertPose(msg.info.yaw,
          msg.info.pitch, transform));
    objectPtr->setProbability(msg.info.probability);
    objectPtr->setTimeFound(timeFound);
    ObjectType::setUpObject(objectPtr, msg);
    objectPtr->initializeObjectFilter();
  }

  template <>
  void ObjectFactory::setUpObject<Motion>(
      const typename Motion::Ptr& objectPtr,
      const typename Motion::Alert& msg,
      const ros::Time& timeFound,
      const tf::Transform& transform)
  {
    objectPtr->setPose(poseFinderPtr_->findAlertPose(msg.yaw,
          msg.pitch, transform));
    objectPtr->setProbability(msg.probability);
    objectPtr->setTimeFound(timeFound);
    objectPtr->initializeObjectFilter();
  }
  template <>
  void ObjectFactory::setUpObject<Sound>(
      const typename Sound::Ptr& objectPtr,
      const typename Sound::Alert& msg,
      const ros::Time& timeFound,
      const tf::Transform& transform)
  {
    objectPtr->setPose(poseFinderPtr_->findAlertPose(msg.yaw,
          msg.pitch, transform));
    objectPtr->setProbability(msg.probability);
    objectPtr->setTimeFound(timeFound);
    objectPtr->initializeObjectFilter();
  }
  template <>
  void ObjectFactory::setUpObject<Co2>(
      const typename Co2::Ptr& objectPtr,
      const typename Co2::Alert& msg,
      const ros::Time& timeFound,
      const tf::Transform& transform)
  {
    objectPtr->setPose(poseFinderPtr_->findAlertPose(msg.yaw,
          msg.pitch, transform));
    objectPtr->setProbability(msg.probability);
    objectPtr->setTimeFound(timeFound);
    objectPtr->initializeObjectFilter();
  }
  template <>
  void ObjectFactory::setUpObject<VisualVictim>(
      const typename VisualVictim::Ptr& objectPtr,
      const typename VisualVictim::Alert& msg,
      const ros::Time& timeFound,
      const tf::Transform& transform)
  {
    objectPtr->setPose(poseFinderPtr_->findAlertPose(msg.yaw,
          msg.pitch, transform));
    objectPtr->setProbability(msg.probability);
    objectPtr->setTimeFound(timeFound);
    objectPtr->initializeObjectFilter();
  }
  template <>
  void ObjectFactory::setUpObject<Obstacle>(
      const typename Obstacle::Ptr& objectPtr,
      const typename Obstacle::Alert& msg,
      const ros::Time& timeFound,
      const tf::Transform& transform)
  {
    double length;
    geometry_msgs::Pose obstaclePose = poseFinderPtr_->findPoseFromPoints(
        msg.pointsYaw, msg.pointsPitch, msg.pointsDepth, transform, &length);
    objectPtr->setPose(obstaclePose);
    if (msg.type == pandora_vision_msgs::ObstacleAlert::SOFT_OBSTACLE) {
      objectPtr->setLength(length);
      objectPtr->setWidth(SOFT_OBSTACLE_WIDTH);
    }
    objectPtr->setProbability(msg.probability);
    objectPtr->setTimeFound(timeFound);
    objectPtr->initializeObjectFilter();
  }

  template <class ObjectType>
  typename ObjectType::PtrVectorPtr ObjectFactory::makeObjects(
      const typename ObjectType::AlertVector& msg)
  {
    currentTransform_ = poseFinderPtr_->lookupTransformFromWorld(
        globalFrame_, msg.header);

    typename ObjectType::PtrVectorPtr objectsVectorPtr(
        new typename ObjectType::PtrVector);
    for (int ii = 0; ii < msg.alerts.size(); ++ii) {
      try
      {
        typename ObjectType::Ptr newObject( new ObjectType );
        newObject->setGlobalFrame(globalFrame_);
        setUpObject<ObjectType>(newObject, msg.alerts[ii],
                                msg.header.stamp, currentTransform_);
        objectsVectorPtr->push_back(newObject);
      }
      catch (pandora_data_fusion_utils::AlertException ex)
      {
        ROS_WARN_NAMED("PANDORA_ALERT_HANDLER",
            "[PANDORA_ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
      }
    }

    return objectsVectorPtr;
  }

  template <>
  typename Obstacle::PtrVectorPtr ObjectFactory::makeObjects<Obstacle>(
      const typename Obstacle::AlertVector& msg)
  {
    currentTransform_ = poseFinderPtr_->lookupTransformFromWorld(
        globalFrame_, msg.header);

    typename Obstacle::PtrVectorPtr obstacleVectorPtr(new typename Obstacle::PtrVector);
    for (int ii = 0; ii < msg.alerts.size(); ++ii) {
      try
      {
        typename Obstacle::Ptr newObstacle;
        switch (msg.alerts[ii].type)
        {
          case pandora_vision_msgs::ObstacleAlert::BARREL:
            newObstacle.reset( new Barrel );
            break;
          case pandora_vision_msgs::ObstacleAlert::SOFT_OBSTACLE:
            newObstacle.reset( new SoftObstacle );
            break;
          case pandora_vision_msgs::ObstacleAlert::HARD_OBSTACLE:
            newObstacle.reset( new HardObstacle );
            break;
          default:
            throw pandora_data_fusion_utils::ObstacleTypeException(
                "Non-registered obstacle type "+boost::to_string(msg.alerts[ii].type));
            break;
        }
        newObstacle->setGlobalFrame(globalFrame_);
        setUpObject<Obstacle>(newObstacle, msg.alerts[ii],
                              msg.header.stamp, currentTransform_);
        obstacleVectorPtr->push_back(newObstacle);
      }
      catch (pandora_data_fusion_utils::AlertException ex)
      {
        ROS_WARN_NAMED("PANDORA_ALERT_HANDLER",
            "[PANDORA_ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
      }
    }

    return obstacleVectorPtr;
  }

  typedef boost::scoped_ptr<ObjectFactory> ObjectFactoryPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_OBJECT_FACTORY_H

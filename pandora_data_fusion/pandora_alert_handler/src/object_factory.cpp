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

#include <string>

#include "alert_handler/object_factory.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    ObjectFactory::ObjectFactory(const MapPtr& map, const std::string& mapType)
    {
      poseFinder_.reset( new PoseFinder(map, mapType) );
    }

    HolePtrVectorPtr ObjectFactory::makeHoles(
        const pandora_vision_msgs::HoleDirectionAlertVector& msg)
    {
      currentHoleTransform_ = poseFinder_->lookupTransformFromWorld(msg.header);

      HolePtrVectorPtr holesVectorPtr( new HolePtrVector );
      for (int ii = 0; ii < msg.holesDirections.size(); ++ii)
      {
        try
        {
          HolePtr newHole( new Hole );
          setUpHole(newHole, msg.holesDirections[ii], msg.header.stamp,
              currentHoleTransform_);
          holesVectorPtr->push_back(newHole);
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
        }
      }

      return holesVectorPtr;
    }

    HazmatPtrVectorPtr ObjectFactory::makeHazmats(
        const pandora_vision_msgs::HazmatAlertVector& msg)
    {
      tf::Transform transform;
      transform = poseFinder_->lookupTransformFromWorld(msg.header);

      HazmatPtrVectorPtr hazmatsVectorPtr( new HazmatPtrVector );
      for (int ii = 0; ii < msg.hazmatAlerts.size(); ++ii)
      {
        try
        {
          HazmatPtr newHazmat( new Hazmat );
          setUpHazmat(newHazmat, msg.hazmatAlerts[ii], msg.header.stamp,
              transform);
          hazmatsVectorPtr->push_back(newHazmat);
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
        }
      }

      return hazmatsVectorPtr;
    }

    QrPtrVectorPtr ObjectFactory::makeQrs(
        const pandora_vision_msgs::QRAlertVector& msg)
    {
      tf::Transform transform;
      transform = poseFinder_->lookupTransformFromWorld(msg.header);

      QrPtrVectorPtr qrsVectorPtr( new QrPtrVector );
      for (int ii = 0; ii < msg.qrAlerts.size(); ++ii)
      {
        try
        {
          QrPtr newQr( new Qr );
          setUpQr(newQr, msg.qrAlerts[ii], msg.header.stamp, transform);
          ROS_DEBUG_NAMED("alert_handler", "[ALERT_HANDLER_OBJECT_FACTORY] Made qr object:");
          ROS_DEBUG_STREAM_NAMED("alert_handler", newQr->getPose());
          qrsVectorPtr->push_back(newQr);
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
        }
      }

      return qrsVectorPtr;
    }

    LandoltcPtrVectorPtr ObjectFactory::makeLandoltcs(
        const pandora_vision_msgs::LandoltcAlertVector& msg)
    {
      tf::Transform transform;
      transform = poseFinder_->lookupTransformFromWorld(msg.header);

      LandoltcPtrVectorPtr landoltcsVectorPtr( new LandoltcPtrVector );
      for (int ii = 0; ii < msg.landoltcAlerts.size(); ++ii)
      {
        try
        {
          LandoltcPtr newLandoltc( new Landoltc );
          setUpLandoltc(newLandoltc, msg.landoltcAlerts[ii], msg.header.stamp,
              transform);
          landoltcsVectorPtr->push_back(newLandoltc);
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
        }
      }

      return landoltcsVectorPtr;
    }

    DataMatrixPtrVectorPtr ObjectFactory::makeDataMatrices(
        const pandora_vision_msgs::DataMatrixAlertVector& msg)
    {
      tf::Transform transform;
      transform = poseFinder_->lookupTransformFromWorld(msg.header);

      DataMatrixPtrVectorPtr dataMatricesVectorPtr( new DataMatrixPtrVector );
      for (int ii = 0; ii < msg.dataMatrixAlerts.size(); ++ii)
      {
        try
        {
          DataMatrixPtr newDataMatrix( new DataMatrix );
          setUpDataMatrix(newDataMatrix, msg.dataMatrixAlerts[ii],
              msg.header.stamp, transform);
          dataMatricesVectorPtr->push_back(newDataMatrix);
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
        }
      }

      return dataMatricesVectorPtr;
    }

    void ObjectFactory::dynamicReconfigForward(float occupiedCellThres,
        float highThres, float lowThres,
        float orientationCircle, float orientationDist)
    {
      poseFinder_->updateParams(occupiedCellThres,
          highThres, lowThres,
          orientationDist, orientationCircle);
    }

    void ObjectFactory::setUpHole(const HolePtr& holePtr,
        const pandora_vision_msgs::HoleDirectionAlert& msg,
        const ros::Time& timeFound,
        const tf::Transform& transform)
    {
      holePtr->setPose(poseFinder_->findAlertPose(msg.info.yaw,
            msg.info.pitch, transform));
      holePtr->setProbability(msg.info.probability);
      holePtr->setTimeFound(timeFound);
      holePtr->setHoleId(msg.holeId);
      holePtr->initializeObjectFilter();
    }

    void ObjectFactory::setUpHazmat(const HazmatPtr& hazmatPtr,
        const pandora_vision_msgs::HazmatAlert& msg,
        const ros::Time& timeFound,
        const tf::Transform& transform)
    {
      hazmatPtr->setPose(poseFinder_->findAlertPose(msg.info.yaw,
            msg.info.pitch, transform));
      hazmatPtr->setProbability(msg.info.probability);
      hazmatPtr->setTimeFound(timeFound);
      hazmatPtr->setPattern(msg.patternType);
      hazmatPtr->initializeObjectFilter();
    }

    void ObjectFactory::setUpQr(const QrPtr& qrPtr,
        const pandora_vision_msgs::QRAlert& msg,
        const ros::Time& timeFound,
        const tf::Transform& transform)
    {
      qrPtr->setPose(poseFinder_->findAlertPose(msg.info.yaw,
            msg.info.pitch, transform));
      qrPtr->setProbability(msg.info.probability);
      qrPtr->setTimeFound(timeFound);
      qrPtr->setContent(msg.QRcontent);
      qrPtr->initializeObjectFilter();
    }

    void ObjectFactory::setUpLandoltc(const LandoltcPtr& landoltcPtr,
        const pandora_vision_msgs::LandoltcAlert& msg,
        const ros::Time& timeFound,
        const tf::Transform& transform)
    {
      landoltcPtr->setPose(poseFinder_->findAlertPose(msg.info.yaw,
            msg.info.pitch, transform));
      landoltcPtr->setProbability(msg.info.probability);
      landoltcPtr->setTimeFound(timeFound);
      landoltcPtr->setAngles(msg.angles);
      landoltcPtr->initializeObjectFilter();
    }

    void ObjectFactory::setUpDataMatrix(const DataMatrixPtr& dataMatrixPtr,
        const pandora_vision_msgs::DataMatrixAlert& msg,
        const ros::Time& timeFound,
        const tf::Transform& transform)
    {
      dataMatrixPtr->setPose(poseFinder_->findAlertPose(msg.info.yaw,
            msg.info.pitch, transform));
      dataMatrixPtr->setProbability(msg.info.probability);
      dataMatrixPtr->setTimeFound(timeFound);
      dataMatrixPtr->setContent(msg.datamatrixContent);
      dataMatrixPtr->initializeObjectFilter();
    }

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

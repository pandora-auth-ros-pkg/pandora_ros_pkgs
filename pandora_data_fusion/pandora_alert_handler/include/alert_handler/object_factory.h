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

#ifndef ALERT_HANDLER_OBJECT_FACTORY_H
#define ALERT_HANDLER_OBJECT_FACTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <string>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>

#include "vision_communications/HolesDirectionsVectorMsg.h"
#include "vision_communications/FaceDirectionMsg.h"
#include "vision_communications/QRAlertsVectorMsg.h"
#include "vision_communications/HazmatAlertsVectorMsg.h"
#include "vision_communications/LandoltcAlertsVectorMsg.h"
#include "vision_communications/DataMatrixAlertsVectorMsg.h"
#include "pandora_common_msgs/GeneralAlertMsg.h"

#include "alert_handler/pose_finder.h"
#include "alert_handler/objects.h"
#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    class ObjectFactory : private boost::noncopyable
    { 
      public:

        ObjectFactory(const MapPtr& map, const std::string& mapType);

        HolePtrVectorPtr makeHoles(
            const vision_communications::HolesDirectionsVectorMsg& msg);
        HazmatPtrVectorPtr makeHazmats(
            const vision_communications::HazmatAlertsVectorMsg& msg);
        QrPtrVectorPtr makeQrs(
            const vision_communications::QRAlertsVectorMsg& msg);
        LandoltcPtrVectorPtr makeLandoltcs(
            const vision_communications::LandoltcAlertsVectorMsg& msg);
        DataMatrixPtrVectorPtr makeDataMatrices(
            const vision_communications::DataMatrixAlertsVectorMsg& msg);
        template <class ObjectType>
          typename ObjectType::PtrVectorPtr makeObjects(
              const pandora_common_msgs::GeneralAlertMsg& msg);

        const tf::Transform& getTransform() const
        {
          return currentTransform_;
        }

        void dynamicReconfigForward(float occupiedCellThres, 
            float highThres, float lowThres,
            float orientationCircle, float orientationDist);

      private:

        /**
         * @brief Sets this Object up according to the info from the Alert.
         * @param objectPtr [const ObjectPtr&] Pointer to Object 
         * variable to be filled.
         * @param msg [const ..._communications::...Msg&] 
         * Incoming ros message containing info.
         * @return void
         */
        void setUpHole(const HolePtr& holePtr, 
            const vision_communications::HoleDirectionMsg& msg);
        void setUpHazmat(const HazmatPtr& hazmatPtr, 
            const vision_communications::HazmatAlertMsg& msg);
        void setUpQr(const QrPtr& qrPtr, 
            const vision_communications::QRAlertMsg& msg,
            ros::Time timeFound);
        void setUpLandoltc(const LandoltcPtr& landoltcPtr, 
            const vision_communications::LandoltcAlertMsg& msg);
        void setUpDataMatrix(const DataMatrixPtr& dataMatrixPtr, 
            const vision_communications::DataMatrixAlertMsg& msg);
        template <class ObjectType>
          void setUpObject(
              const typename ObjectType::Ptr& objectPtr, 
              const pandora_common_msgs::GeneralAlertMsg& msg);

      private:

        tf::Transform currentTransform_;

        PoseFinderPtr poseFinder_;

    };

    template <class ObjectType>
      typename ObjectType::PtrVectorPtr ObjectFactory::makeObjects(
          const pandora_common_msgs::GeneralAlertMsg& msg)
      {
        currentTransform_ = poseFinder_->lookupTransformFromWorld( msg.header );

        typename ObjectType::PtrVectorPtr objectsVectorPtr(
            new typename ObjectType::PtrVector);
        try
        {
          typename ObjectType::Ptr newObject( new ObjectType );
          setUpObject<ObjectType>( newObject, msg );
          objectsVectorPtr->push_back( newObject );
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
        }

        return objectsVectorPtr;
      }

    template <class ObjectType>
      void ObjectFactory::setUpObject(
          const typename ObjectType::Ptr& objectPtr, 
          const pandora_common_msgs::GeneralAlertMsg& msg)
      {
        objectPtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
              msg.pitch, currentTransform_) );
        objectPtr->setProbability( msg.probability );
        objectPtr->initializeObjectFilter();
      }

    typedef boost::scoped_ptr< ObjectFactory > ObjectFactoryPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_FACTORY_H

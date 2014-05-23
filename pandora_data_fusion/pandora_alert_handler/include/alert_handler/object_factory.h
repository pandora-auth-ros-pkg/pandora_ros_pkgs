// "Copyright [year] <Copyright Owner>"

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
#include "vision_communications/HolesPositionsVectorMsg.h"
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
              "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
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

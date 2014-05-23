// "Copyright [year] <Copyright Owner>"

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

    class ObjectHandler : private boost::noncopyable
    {
      public:

        ObjectHandler(const VictimListConstPtr& victimsToGoList,
            const VictimListConstPtr& victimsVisited);

        void handleHoles(const HolePtrVectorPtr& newHoles, 
            const tf::Transform& transform);
        void handleQrs(const QrPtrVectorPtr& newQrs); 
        template <class ObjectType> 
          void handleObjects( 
              const typename ObjectType::PtrVectorPtr& objectsPtr);

        void updateParams(float sensor_range, float victim_cluster_radius);

      private:

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
        if(ObjectType::getObjectType() != Thermal::getObjectType() && 
            ObjectType::getObjectType() != Hazmat::getObjectType())
        {
          keepValidVerificationObjects<ObjectType>(newObjects);
        }
        for(int ii = 0; ii < newObjects->size(); ++ii)
        {
          int objectScore = ObjectType::getList()->add(newObjects->at(ii));
          if(objectScore)
          {
            std_msgs::Int32 updateScoreMsg;
            roboCupScore_ += objectScore;
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

        while(iter != objectsPtr->end())
        {
          bool valid = false;
          valid = victimsToGoList_->isObjectPoseInList((*iter), VICTIM_CLUSTER_RADIUS);
          // valid = valid || 
          //   victimsVisitedList_->isObjectPoseInList((*iter), VICTIM_CLUSTER_RADIUS);
          if(!valid)
          {
            ROS_DEBUG_NAMED("object_handler",
                "[OBJECT_HANDLER %d] Deleting not valid object...", __LINE__);
            iter = objectsPtr->erase(iter);
          }
          else
          {
            ++iter;
          }
        }
      }

    template<>
      void ObjectHandler::keepValidVerificationObjects<Sound>(
          const Sound::PtrVectorPtr& objectsPtr)
      {
        SoundPtrVector::iterator iter = objectsPtr->begin();

        while(iter != objectsPtr->end())
        {
          bool invalid = false;
          for(VictimList::const_iterator it = victimsToGoList_->begin();
              it != victimsToGoList_->end(); it++)
          {
            invalid = Utils::distanceBetweenPoints2D((*iter)->getPose().position, 
                (*it)->getPose().position) >= VICTIM_CLUSTER_RADIUS;
            if(invalid)
              break;
          }
          for(VictimList::const_iterator it = victimsVisitedList_->begin();
              it != victimsVisitedList_->end(); it++)
          {
            invalid = invalid || Utils::distanceBetweenPoints2D((*iter)->getPose().position, 
                (*it)->getPose().position) >= VICTIM_CLUSTER_RADIUS;
            if(invalid)
              break;
          }
          if(invalid)
          {
            ROS_DEBUG_NAMED("object_handler",
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

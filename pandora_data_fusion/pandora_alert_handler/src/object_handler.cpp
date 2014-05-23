// "Copyright [year] <Copyright Owner>"

#include "alert_handler/object_handler.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    ObjectHandler::ObjectHandler(const VictimListConstPtr& victimsToGoList,
        const VictimListConstPtr& victimsVisitedList) : 
      victimsToGoList_(victimsToGoList),
      victimsVisitedList_(victimsVisitedList)
    {
      Hole::setType("hole");
      Hazmat::setType("hazmat");
      Qr::setType("qr");
      Thermal::setType("thermal");
      Face::setType("face");
      Motion::setType("motion");
      Sound::setType("sound");
      Co2::setType("co2");

      roboCupScore_ = 0;

      std::string param;

      if (ros::param::get("published_topic_names/qr_notification", param))
      {
        qrPublisher_ = ros::NodeHandle().
          advertise<pandora_data_fusion_msgs::QrNotificationMsg>(param, 10);
      }
      else
      {
        ROS_FATAL("qr_notification topic name param not found");
        ROS_BREAK();
      }

      if (ros::param::get("published_topic_names/robocup_score", param))
      {
        scorePublisher_ = ros::NodeHandle().
          advertise<std_msgs::Int32>(param, 10);
      }
      else
      {
        ROS_FATAL("robocup_score topic name param not found");
        ROS_BREAK();
      }
    }

    void ObjectHandler::handleHoles(const HolePtrVectorPtr& newHoles,
        const tf::Transform& transform)
    {
      keepValidHoles(newHoles, transform);

      for(int ii = 0; ii < newHoles->size(); ++ii)
      {
        Hole::getList()->add( newHoles->at(ii) );
      }
    }

    void ObjectHandler::handleQrs(const QrPtrVectorPtr& newQrs) 
    {
      for(int ii = 0; ii < newQrs->size(); ++ii)
      {
        int qrScore = Qr::getList()->add(newQrs->at(ii));
        if(qrScore)
        {
          pandora_data_fusion_msgs::QrNotificationMsg newQrNofifyMsg;
          newQrNofifyMsg.header.stamp = newQrs->at(ii)->getTimeFound();
          newQrNofifyMsg.x = newQrs->at(ii)->getPose().position.x;
          newQrNofifyMsg.y = newQrs->at(ii)->getPose().position.y;
          newQrNofifyMsg.content = newQrs->at(ii)->getContent();
          qrPublisher_.publish(newQrNofifyMsg);
          std_msgs::Int32 updateScoreMsg;
          roboCupScore_ += qrScore;
          updateScoreMsg.data = roboCupScore_;
          scorePublisher_.publish(updateScoreMsg);
        }
      }
    }

    void ObjectHandler::keepValidHoles(const HolePtrVectorPtr& holesPtr,
        const tf::Transform& transform)
    {
      tf::Vector3 origin = transform.getOrigin();
      geometry_msgs::Point framePosition = Utils::vector3ToPoint(origin);

      HolePtrVector::iterator iter = holesPtr->begin();

      while(iter != holesPtr->end())
      {
        bool invalid = !Utils::arePointsInRange((*iter)->getPose().position,
            framePosition, SENSOR_RANGE );

        if(invalid)
        {
          ROS_DEBUG_NAMED("object_handler",
              "[OBJECT_HANDLER %d] Deleting not valid hole...", __LINE__);
          iter = holesPtr->erase(iter);
        }
        else
        {
          ++iter;
        }
      }
    }

    void ObjectHandler::updateParams(float sensor_range, float victim_cluster_radius)
    {
      SENSOR_RANGE = sensor_range;
      VICTIM_CLUSTER_RADIUS = victim_cluster_radius;
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


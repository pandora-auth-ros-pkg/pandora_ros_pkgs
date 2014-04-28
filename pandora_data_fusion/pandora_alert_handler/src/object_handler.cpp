// "Copyright [year] <Copyright Owner>"

#include "alert_handler/object_handler.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

ObjectHandler::ObjectHandler(HoleListPtr holeListPtr, QrListPtr qrListPtr,
                            HazmatListPtr hazmatListPtr, TpaListPtr tpaListPtr) :
  holeListPtr_(holeListPtr),
  qrListPtr_(qrListPtr),
  hazmatListPtr_(hazmatListPtr),
  tpaListPtr_(tpaListPtr)
{
  roboCupScore_ = 0;

  std::string param;

  if (ros::param::get("published_topic_names/qr_notification", param))
  {
    qrPublisher_ = ros::NodeHandle().
      advertise<data_fusion_communications::QrNotificationMsg>(param, 10);
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

  for (int ii = 0; ii < newHoles->size(); ++ii)
  {
    holeListPtr_->add( newHoles->at(ii) );
  }
}

void ObjectHandler::handleQrs(const QrPtrVectorPtr& newQrs, 
  const tf::Transform& transform, bool eraseHoles)
{
  for (int ii = 0; ii < newQrs->size(); ++ii)
  {
    int qrScore = qrListPtr_->add( newQrs->at(ii) );
    if (qrScore)
    {
      data_fusion_communications::QrNotificationMsg newQrNofifyMsg;
      newQrNofifyMsg.header.stamp = ros::Time::now();
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

void ObjectHandler::handleHazmats(const HazmatPtrVectorPtr& newHazmats,
  const tf::Transform& transform)
{
  for (int ii = 0; ii < newHazmats->size(); ++ii)
  {
    int hazmatScore = hazmatListPtr_->add( newHazmats->at(ii) );
    if (hazmatScore)
    {
      std_msgs::Int32 updateScoreMsg;
      roboCupScore_ += hazmatScore;
      updateScoreMsg.data = roboCupScore_;
      scorePublisher_.publish(updateScoreMsg);
    }
  }
}

void ObjectHandler::handleTpas(const TpaPtrVectorPtr& newTpas,
  const tf::Transform& transform)
{
  for (int ii = 0; ii < newTpas->size(); ++ii)
  {
    int tpaScore = tpaListPtr_->add( newTpas->at(ii) );
    if (tpaScore)
    {
      std_msgs::Int32 updateScoreMsg;
      roboCupScore_ += tpaScore;
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

  HolePtrVector::iterator iter;
  iter = holesPtr->begin();

  while (iter != holesPtr->end() )
  {
    bool isInRange = !Utils::arePointsInRange((*iter)->getPose().position,
      framePosition, SENSOR_RANGE );

    bool inValid = isInRange;

    if ( inValid )
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

void ObjectHandler::updateParams(float sensorRange)
{
  SENSOR_RANGE = sensorRange;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


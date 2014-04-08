// "Copyright [year] <Copyright Owner>"

#include "alert_handler/object_handler.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

ObjectHandler::ObjectHandler(HoleListPtr holeListPtr, QrListPtr qrListPtr,
                            HazmatListPtr hazmatListPtr, TpaListPtr tpaListPtr,
                            float sensorRange, float qrClosestAlert,
                            float hazmatClosestalert) :
  holeListPtr_(holeListPtr),
  qrListPtr_(qrListPtr),
  hazmatListPtr_(hazmatListPtr),
  tpaListPtr_(tpaListPtr),
  SENSOR_RANGE(sensorRange),
  QR_CLOSEST_ALERT(qrClosestAlert),
  HAZMAT_CLOSEST_ALERT(hazmatClosestalert)
{
  std::string param;

  if (ros::param::get("published_topic_names/qr_notification", param))
  {
    qrPublisher_ = ros::NodeHandle().advertise<data_fusion_communications::QrNotificationMsg>(param, 10);
  }
  else
  {
    ROS_FATAL("qr_notification topic name param not found");
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
    bool isQrNew = qrListPtr_->add( newQrs->at(ii) );
    if (isQrNew)
    {
      data_fusion_communications::QrNotificationMsg newQrNofifyMsg;
      newQrNofifyMsg.header.stamp = ros::Time::now();
      newQrNofifyMsg.x = newQrs->at(ii)->getPose().position.x;
      newQrNofifyMsg.y = newQrs->at(ii)->getPose().position.y;
      newQrNofifyMsg.content = newQrs->at(ii)->getContent();
      qrPublisher_.publish(newQrNofifyMsg);
    }
    if (eraseHoles)
    {
      holeListPtr_->removeInRangeOfObject(newQrs->at(ii), QR_CLOSEST_ALERT);
    }
  }
}

void ObjectHandler::handleHazmats(const HazmatPtrVectorPtr& newHazmats,
  const tf::Transform& transform)
{
  for (int ii = 0; ii < newHazmats->size(); ++ii)
  {
    hazmatListPtr_->add( newHazmats->at(ii) );
    // _holeListPtr->removeInRangeOfObject(newHazmats.at(ii),
      // HAZMAT_CLOSEST_ALERT);
  }
}

void ObjectHandler::handleTpas(const TpaPtrVectorPtr& newTpas,
  const tf::Transform& transform)
{
  for (int ii = 0; ii < newTpas->size(); ++ii)
  {
    tpaListPtr_->add( newTpas->at(ii) );
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
    bool isQr = isHoleQr(*iter);
    bool isHazmat = isHoleHazmat(*iter);
    bool isInRange = !Utils::arePointsInRange((*iter)->getPose().position,
      framePosition, SENSOR_RANGE );

    bool inValid = isQr || isHazmat || isInRange;

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

bool ObjectHandler::isHoleQr(const HoleConstPtr& hole)
{
  return qrListPtr_->isObjectPoseInList(hole, QR_CLOSEST_ALERT);
}

bool ObjectHandler::isHoleHazmat(const HoleConstPtr& hole)
{
  return hazmatListPtr_->isObjectPoseInList(hole, HAZMAT_CLOSEST_ALERT);
}

void ObjectHandler::updateParams(
  float sensorRange,
  float qrClosestAlert,
  float hazmatClosestalert )
{
  SENSOR_RANGE = sensorRange;
  QR_CLOSEST_ALERT = qrClosestAlert;
  HAZMAT_CLOSEST_ALERT = hazmatClosestalert;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


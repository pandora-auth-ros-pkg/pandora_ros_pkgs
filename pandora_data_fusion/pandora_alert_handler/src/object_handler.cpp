// "Copyright [year] <Copyright Owner>"

#include "alert_handler/object_handler.h"

ObjectHandler::ObjectHandler(HoleListPtr holeListPtr, QrListPtr qrListPtr,
                            HazmatListPtr hazmatListPtr, TpaListPtr tpaListPtr,
                            float sensorRange, float qrClosestAlert,
                            float hazmatClosestalert) :
  _qrListPtr(qrListPtr),
  _hazmatListPtr(hazmatListPtr),
  _holeListPtr(holeListPtr),
  _tpaListPtr(tpaListPtr),
  SENSOR_RANGE(sensorRange),
  QR_CLOSEST_ALERT(qrClosestAlert),
  HAZMAT_CLOSEST_ALERT(hazmatClosestalert) {
  _qrPublisher = ros::NodeHandle().advertise<
                  data_fusion_communications::QrNotificationMsg>
                 ("/data_fusion/qr_notification", 10);
}

void ObjectHandler::handleHoles(HolePtrStdVector newHoles,
  tf::Transform transform) {

  keepValidHoles(&newHoles, transform);

  for (int ii = 0; ii < newHoles.size(); ii++) {
    _holeListPtr->add( newHoles.at(ii) );
  }

}



void ObjectHandler::handleQrs(QrPtrStdVector newQrs, tf::Transform transform,
  bool eraseHoles) {

  for (int ii = 0; ii < newQrs.size(); ii++) {

    bool isQrNew = _qrListPtr->add( newQrs.at(ii) );
    if (isQrNew) {
      data_fusion_communications::QrNotificationMsg newQrNofifyMsg;
      newQrNofifyMsg.header.stamp = ros::Time::now();
      newQrNofifyMsg.x = newQrs.at(ii)->getPose().position.x;
      newQrNofifyMsg.y = newQrs.at(ii)->getPose().position.y;
      newQrNofifyMsg.content = newQrs.at(ii)->getContent();
      _qrPublisher.publish(newQrNofifyMsg);
    }
    if (eraseHoles) {
      _holeListPtr->removeInRangeOfObject(newQrs.at(ii), QR_CLOSEST_ALERT);
    }
  }



}

void ObjectHandler::handleHazmats(HazmatPtrStdVector newHazmats,
  tf::Transform transform) {

  for (int ii = 0; ii < newHazmats.size(); ii++) {
    _hazmatListPtr->add( newHazmats.at(ii) );
    //~ _holeListPtr->removeInRangeOfObject(newHazmats.at(ii),
      //~ HAZMAT_CLOSEST_ALERT);
  }

}

void ObjectHandler::handleTpas(TpaPtrStdVector newTpas,
  tf::Transform transform) {

  for (int ii = 0; ii < newTpas.size(); ii++) {
    _tpaListPtr->add( newTpas.at(ii) );
  }

}

void ObjectHandler::keepValidHoles(HolePtrStdVector* holes,
    tf::Transform transform) {
  tf::Vector3 origin = transform.getOrigin();
  geometry_msgs::Point framePosition = Utils::vector3ToPoint(origin);

  HolePtrStdVector::iterator iter;
  iter = holes->begin();

  while (iter != holes->end() ) {

    bool isQr = isHoleQr(*iter);
    bool isHazmat = isHoleHazmat(*iter);
    bool isInRange = !Utils::arePointsInRange((*iter)->getPose().position,
      framePosition, SENSOR_RANGE );

    bool inValid = isQr || isHazmat || isInRange;

    if ( inValid ) {
      ROS_DEBUG_NAMED("object_handler",
        "[OBJECT_HANDLER %d] Deleting not valid hole...", __LINE__);
      iter = holes->erase(iter);
    } else {
      ++iter;
    }

  }

}

bool ObjectHandler::isHoleQr(ObjectPtr hole) {
  return _qrListPtr->isObjectPoseInList(hole, QR_CLOSEST_ALERT);
}

bool ObjectHandler::isHoleHazmat(ObjectPtr hole) {
  return _hazmatListPtr->isObjectPoseInList(hole, HAZMAT_CLOSEST_ALERT);
}


void ObjectHandler::updateParams(
  float sensorRange,
  float qrClosestAlert,
  float hazmatClosestalert ) {
  SENSOR_RANGE = sensorRange;
  QR_CLOSEST_ALERT = qrClosestAlert;
  HAZMAT_CLOSEST_ALERT = hazmatClosestalert;
}

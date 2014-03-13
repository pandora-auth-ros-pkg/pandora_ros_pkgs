// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_HANDLER_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_HANDLER_H_

#include "alert_handler/object_list.h"
#include "data_fusion_communications/QrNotificationMsg.h"

class ObjectHandler {

 public:

  ObjectHandler(HoleListPtr holeListPtr, QrListPtr qrListPtr,
                HazmatListPtr hazmatListPtr , TpaListPtr tpaListPtr ,
                float sensorRange = 2.5,
                float qrClosestAlert = 0.5,
                float hazmatClosestalert = 0.5);

  void handleHoles(HolePtrStdVector holes, tf::Transform transform);
  void handleQrs(QrPtrStdVector newQrs,
    tf::Transform transform, bool eraseHoles);
  void handleHazmats(HazmatPtrStdVector newHazmats, tf::Transform transform);
  void handleTpas(TpaPtrStdVector newTpas, tf::Transform transform);

  bool isHoleQr(ObjectPtr hole);
  bool isHoleHazmat(ObjectPtr hole);

  void updateParams(float sensor_range,
     float qrClosestAlert, float hazmatClosestalert);

 private:

  void keepValidHoles(HolePtrStdVector* holesPtr,
     tf::Transform cameraTransform);

 private:

  ros::Publisher _qrPublisher;

  QrListPtr _qrListPtr;
  HazmatListPtr _hazmatListPtr;
  HoleListPtr _holeListPtr;
  TpaListPtr _tpaListPtr;

  float SENSOR_RANGE;
  float QR_CLOSEST_ALERT;
  float HAZMAT_CLOSEST_ALERT;

};

typedef boost::shared_ptr< ObjectHandler >  ObjectHandlerPtr;

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_HANDLER_H_

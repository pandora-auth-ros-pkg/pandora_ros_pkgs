// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_OBJECT_HANDLER_H
#define ALERT_HANDLER_OBJECT_HANDLER_H

#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

#include <std_msgs/Int32.h>

#include "data_fusion_communications/QrNotificationMsg.h"

#include "alert_handler/object_list.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class ObjectHandler : private boost::noncopyable
{
 public:

  ObjectHandler(HoleListPtr holeListPtr, QrListPtr qrListPtr,
                HazmatListPtr hazmatListPtr, TpaListPtr tpaListPtr);

  void handleHoles(const HolePtrVectorPtr& newHoles, const tf::Transform& transform);
  void handleQrs(const QrPtrVectorPtr& newQrs, 
      const tf::Transform& transform, bool eraseHoles);
  void handleHazmats(const HazmatPtrVectorPtr& newHazmats, const tf::Transform& transform);
  void handleTpas(const TpaPtrVectorPtr& newTpas, const tf::Transform& transform);

  void updateParams(float sensor_range);

 private:

  void keepValidHoles(const HolePtrVectorPtr& holesPtr,
     const tf::Transform& cameraTransform);

 private:

  ros::Publisher qrPublisher_;
  ros::Publisher scorePublisher_;

  QrListPtr qrListPtr_;
  HazmatListPtr hazmatListPtr_;
  HoleListPtr holeListPtr_;
  TpaListPtr tpaListPtr_;

  int roboCupScore_;
  
  float SENSOR_RANGE;

};

typedef boost::scoped_ptr< ObjectHandler >  ObjectHandlerPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_HANDLER_H

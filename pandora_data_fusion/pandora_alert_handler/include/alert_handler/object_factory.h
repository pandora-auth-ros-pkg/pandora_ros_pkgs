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
#include "data_fusion_communications/ThermalDirectionAlertMsg.h"

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
  TpaPtrVectorPtr makeTpas(
      const data_fusion_communications::ThermalDirectionAlertMsg& msg);
  HazmatPtrVectorPtr makeHazmats(
      const vision_communications::HazmatAlertsVectorMsg& msg);
  QrPtrVectorPtr makeQrs(
      const vision_communications::QRAlertsVectorMsg& msg);

  const tf::Transform& getTransform() const
  {
    return currentTransform_;
  }

  void dynamicReconfigForward(float occupiedCellThres, 
      float highThres, float lowThres, float approachDist, 
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
  void setUpObject(const HolePtr& holePtr, 
      const vision_communications::HoleDirectionMsg& msg);
  void setUpObject(const TpaPtr& tpaPtr, 
      const data_fusion_communications::ThermalDirectionAlertMsg& msg);
  void setUpObject(const HazmatPtr& hazmatPtr, 
      const vision_communications::HazmatAlertMsg& msg);
  void setUpObject(const QrPtr& qrPtr, 
      const vision_communications::QRAlertMsg& msg);

 private:

  tf::Transform currentTransform_;
  
  PoseFinderPtr poseFinder_;

};

typedef boost::scoped_ptr< ObjectFactory > ObjectFactoryPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_FACTORY_H

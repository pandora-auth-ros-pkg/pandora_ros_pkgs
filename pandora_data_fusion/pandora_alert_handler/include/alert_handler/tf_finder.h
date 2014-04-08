// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_TF_FINDER_H
#define ALERT_HANDLER_TF_FINDER_H

#include <string>
#include <boost/utility.hpp>

#include "alert_handler/exceptions.h"
#include "alert_handler/utils.h"
#include "alert_handler/tf_listener.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class TfFinder : private boost::noncopyable
{
 public:
 
  static TfListener* newTfListener(const std::string& type)
  {
    if( type == "SLAM" )
    {
      return new RosTfListener;
    }
    if( type == "TEST" )
    {
      return new TfListener;
    }
    else
    {
      ROS_ERROR("[ALERT_HANDLER %d]Runtime error: map_type not found.", __LINE__);
      return NULL;
    }
  }

};

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_TF_FINDER_H

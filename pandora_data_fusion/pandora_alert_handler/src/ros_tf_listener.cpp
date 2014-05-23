// Copyright [year] <Copyright Owner>

#include "alert_handler/tf_listener.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    RosTfListener::RosTfListener()
    {
      tf::StampedTransform tfTransform;

      waitForTransform("/map", "/map",
          ros::Time(0), ros::Duration(1));

      lookupTransform("/map", "/map", 
          ros::Time(0), tfTransform);
    }

    bool RosTfListener::waitForTransform(const std::string& target_frame, 
        const std::string& source_frame, const ros::Time& time, 
        const ros::Duration& timeout, const ros::Duration& polling_sleep_duration, 
        std::string* error_msg) const
    {
      bool flag;
      try
      {
        flag = listener.waitForTransform(target_frame, source_frame, time, 
            timeout, polling_sleep_duration, error_msg);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("[ALERT_HANDLER %d]%s", __LINE__, ex.what());
        throw AlertException(
            "Something went wrong with tf, ignoring current message");
      }
      return flag;
    }

    void RosTfListener::lookupTransform(const std::string& target_frame, 
        const std::string& source_frame, const ros::Time& time, 
        tf::StampedTransform& transform) const
    {
      try
      {
        listener.lookupTransform(target_frame, source_frame, time, transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("[ALERT_HANDLER %d]%s", __LINE__, ex.what());
        throw AlertException(
            "Something went wrong with tf, ignoring current message");
      }
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


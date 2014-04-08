// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_TF_LISTENER_H
#define ALERT_HANDLER_TF_LISTENER_H

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class TfListener
{
 public:

  typedef boost::shared_ptr<TfListener> Ptr;
  typedef boost::shared_ptr<TfListener const> ConstPtr;

  TfListener() {}
  
  virtual bool waitForTransform(const std::string& target_frame, 
                    const std::string& source_frame, const ros::Time& time, 
                    const ros::Duration& timeout, 
                    const ros::Duration& polling_sleep_duration = ros::Duration(0.01), 
                    std::string* error_msg = NULL) const
                    {
                       return true;
                    }
  virtual void lookupTransform(const std::string& target_frame, 
                    const std::string& source_frame, const ros::Time& time, 
                    tf::StampedTransform& transform) const
                    {
                      transform.setOrigin(tf::Vector3(5, 5, 0.3));
                      transform.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
                    }
};

typedef TfListener::Ptr TfListenerPtr;
typedef TfListener::ConstPtr TfListenerConstPtr;

class RosTfListener: public TfListener
{
 public:

  RosTfListener();
  
  bool waitForTransform(const std::string& target_frame, 
                        const std::string& source_frame, const ros::Time& time, 
                        const ros::Duration& timeout, 
                        const ros::Duration& polling_sleep_duration = ros::Duration(0.01), 
                        std::string* error_msg = NULL) const;
  void lookupTransform(const std::string& target_frame, 
                       const std::string& source_frame, const ros::Time& time, 
                       tf::StampedTransform& transform) const;
  
 private:
   
  tf::TransformListener listener;

};

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_TF_LISTENER_H

// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Tpa::Tpa()
{
  type_ = "tpa";
}

PoseStamped Tpa::getPoseStamped() const
{
  PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "tpa_" + boost::to_string(id_);
  return objPose;
}

void Tpa::getVisualization(visualization_msgs::MarkerArray* markers) const
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "Tpa";
  marker.id = id_;

  marker.pose = pose_;

  marker.type = visualization_msgs::Marker::SPHERE;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.545;
  marker.color.g = 0.412;
  marker.color.b = 0.08;
  marker.color.a = 0.7;

  markers->markers.push_back(marker);
}

bool Tpa::isSameObject(const ObjectConstPtr& object, float distance) const
{
  bool cond = false;

  if (!object->getType().compare(type_))
  {
      cond = Object::isSameObject(object, distance);
  }

  return cond;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


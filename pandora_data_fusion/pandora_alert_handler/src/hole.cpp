// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Hole::Hole()
{
  type_ = "hole";
}

geometry_msgs::PoseStamped Hole::getPoseStamped() const
{
  geometry_msgs::PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "hole_" + boost::to_string(id_);
  return objPose;
}

void Hole::getVisualization(visualization_msgs::MarkerArray* markers) const
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "Hole";
  marker.id = id_;

  marker.pose = pose_;

  marker.type = visualization_msgs::Marker::SPHERE;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 0.7;

  markers->markers.push_back(marker);
}

bool Hole::isSameObject(const ObjectConstPtr& object, float distance) const
{
  bool cond = false;

  if (object->getType().compare(std::string("tpa")))
  {
    cond = Object::isSameObject(object, distance);
  }

  return cond;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


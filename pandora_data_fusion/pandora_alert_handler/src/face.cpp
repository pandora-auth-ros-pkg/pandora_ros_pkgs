// "Copyright [year] <Copyright Owner>"

#include "alert_handler/face.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    Face::Face() {}

    void Face::getVisualization(visualization_msgs::
        MarkerArray* markers) const
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = getFrameId();
      marker.header.stamp = ros::Time::now();
      marker.ns = "Face";
      marker.id = id_;

      marker.pose = pose_;

      marker.type = visualization_msgs::Marker::SPHERE;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 1;
      marker.color.g = 0.65;
      marker.color.b = 0;
      marker.color.a = 0.7;

      markers->markers.push_back(marker);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


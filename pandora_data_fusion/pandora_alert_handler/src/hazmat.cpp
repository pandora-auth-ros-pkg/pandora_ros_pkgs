// "Copyright [year] <Copyright Owner>"

#include "alert_handler/hazmat.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    Hazmat::Hazmat() {}

    PoseStamped Hazmat::getPoseStamped() const
    {
      PoseStamped objPose = Object<Hazmat>::getPoseStamped();
      objPose.header.frame_id = objPose.header.frame_id + "_" + 
        boost::to_string(pattern_);
      return objPose;
    }

    bool Hazmat::isSameObject(const ObjectConstPtr& object) const
    {
      bool cond = Object<Hazmat>::isSameObject(object) 
        && pattern_ == 
        boost::dynamic_pointer_cast<const Hazmat>(object)->getPattern();

      return cond;
    }

    void Hazmat::fillGeotiff(pandora_data_fusion_msgs::
        DatafusionGeotiffSrv::Response* res) const
    {
      res->hazmatx.push_back( pose_.position.x );
      res->hazmaty.push_back( pose_.position.y );
      res->pattern.push_back( pattern_ );
    }

    void Hazmat::getVisualization(visualization_msgs::
        MarkerArray* markers) const
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = getFrameId();
      marker.header.stamp = ros::Time::now();
      marker.ns = "Hazmat";
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


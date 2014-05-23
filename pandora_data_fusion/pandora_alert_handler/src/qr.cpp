// "Copyright [year] <Copyright Owner>"

#include "alert_handler/qr.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    Qr::Qr() {}

    PoseStamped Qr::getPoseStamped() const
    {
      PoseStamped objPose = Object<Qr>::getPoseStamped();
      objPose.header.frame_id = objPose.header.frame_id + "_" + content_;
      return objPose;
    }

    bool Qr::isSameObject(const ObjectConstPtr& object) const
    {
      bool cond = Object<Qr>::isSameObject(object) 
        && !content_.compare(
            boost::dynamic_pointer_cast<const Qr>(object)->getContent());

      return cond;
    }

    void Qr::fillGeotiff(pandora_data_fusion_msgs::
        DatafusionGeotiffSrv::Response* res) const
    {
      res->qrx.push_back( pose_.position.x );
      res->qry.push_back( pose_.position.y );
      res->qrworldx.push_back( pose_.position.x );
      res->qrworldy.push_back( pose_.position.y );
      res->qrcontent.push_back(content_);
      res->qrtimestamp.push_back(timeFound_);
    }

    void Qr::getVisualization(visualization_msgs::
        MarkerArray* markers) const
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = getFrameId();
      marker.header.stamp = ros::Time::now();
      marker.ns = "Qr";
      marker.id = id_;

      marker.pose = pose_;

      marker.type = visualization_msgs::Marker::SPHERE;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 0.7;

      markers->markers.push_back(marker);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


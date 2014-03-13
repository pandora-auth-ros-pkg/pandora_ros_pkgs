// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

Object::Object() {
  counter_ = 0;
  legit_ = false;
  frame_id_ = "/world";
}

geometry_msgs::PoseStamped Object::getPoseStamped() const {

  geometry_msgs::PoseStamped objPoseStamped;
  objPoseStamped.pose = pose_;
  return objPoseStamped;
}

bool Object::isSameObject(const ConstPtr& object, float distance) const {

  return 
    Utils::distanceBetweenPoints3D(pose_.position, object->getPose().position)
      < distance;

}

Qr::Qr() {
  type_ = "qr";
  timeFound_ = ros::Time::now();
}

geometry_msgs::PoseStamped Qr::getPoseStamped() const {

  geometry_msgs::PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "qr_" + boost::to_string(id_) + "_" + content_;
  return objPose;
}

bool Qr::isSameObject(const ObjectConstPtr& object, float distance) const {

  bool cond = Object::isSameObject(object, distance);

  if (!object->getType().compare(type_)) {
    cond = cond && !content_.compare(
             boost::dynamic_pointer_cast<const Qr>(object)->getContent() );
  }

  return cond;

}

void Qr::fillGeotiff(
  data_fusion_communications::DatafusionGeotiffSrv::Response* res) const {

  PixelCoords coords = Utils::pointToPixelCoords(pose_.position);
  res->qrx.push_back( coords.getXCoord() );
  res->qry.push_back( coords.getYCoord() );
  res->qrworldx.push_back( pose_.position.x );
  res->qrworldy.push_back( pose_.position.y );
  res->qrcontent.push_back(content_);
  res->qrtimestamp.push_back(timeFound_);
}

void  Qr::getVisualization(visualization_msgs::MarkerArray* markers) const {

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "Hazmat";
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

Hazmat::Hazmat() {
  type_ = "hazmat";
}

geometry_msgs::PoseStamped Hazmat::getPoseStamped() const {

  geometry_msgs::PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "hazmat_" + boost::to_string(id_) + "_" +
                            boost::to_string(pattern_);
  return objPose;
}

bool Hazmat::isSameObject(const ObjectConstPtr& object, float distance) const {

  bool cond = Object::isSameObject(object, distance);

  if (!object->getType().compare(type_)) {
    cond = cond &&
    pattern_ == boost::dynamic_pointer_cast<const Hazmat>(object)->getPattern();
  }

  return cond;

}

void Hazmat::fillGeotiff(
  data_fusion_communications::DatafusionGeotiffSrv::Response* res) const {

  PixelCoords coords = Utils::pointToPixelCoords( pose_.position );
  res->hazmatx.push_back( coords.getXCoord() );
  res->hazmaty.push_back( coords.getYCoord() );
  res->pattern.push_back( pattern_ );
}

void Hazmat::getVisualization(visualization_msgs::MarkerArray* markers) const {

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/world";
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

Hole::Hole() {
  type_ = "hole";
}

geometry_msgs::PoseStamped Hole::getPoseStamped() const {

  geometry_msgs::PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "hole_" + boost::to_string(id_);
  return objPose;
}

void Hole::getVisualization(visualization_msgs::MarkerArray* markers) const {

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

Tpa::Tpa() {
  type_ = "tpa";
}

geometry_msgs::PoseStamped Tpa::getPoseStamped() const {

  geometry_msgs::PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "tpa_" + boost::to_string(id_);
  return objPose;
}

void Tpa::getVisualization(visualization_msgs::MarkerArray* markers) const {

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

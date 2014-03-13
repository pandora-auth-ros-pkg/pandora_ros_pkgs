// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_POSE_FINDER_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_POSE_FINDER_H_

#include <utility> 
#include <vector> 

#include <tf/transform_broadcaster.h>

#include "alert_handler/objects.h"
#include "alert_handler/utils.h"

typedef unsigned char** Map;

class PoseFinder {

 public:

  PoseFinder(const Map& map,
      float heightHighThres = 1.2, float heightLowThres = 0,
      float approachDist = 0.5, int orientationDist = 20,
      int orientationCircle = 10);
  Pose findAlertPose(float alertYaw, float alertPitch,
      tf::Transform tfTransform);
  tf::Transform lookupTransformFromWorld(std_msgs::Header header);

  void updateParams(float heightHighThres, float heightLowThres,
      float approachDist,
      int orientationDist, int orientationCircle);

 private:

  PixelCoords positionOnWall(Point startPoint, float angle);
  float calcHeight(float alertPitch, float height, float distFromAlert);
  geometry_msgs::Quaternion findNormalVectorOnWall(Point framePoint,
      Point alertPoint);
  std::pair<PixelCoords, PixelCoords> findDiameterEndPointsOnWall(
      std::vector<PixelCoords> points);

  void publishVisionTransform(float alertYaw, float alertPitch,
      tf::Transform worldHeadCameraTransform);
      
 private:

  const Map& _map;
  tf::TransformListener _listener;
  tf::TransformBroadcaster victimFrameBroadcaster;

  //params
  int ORIENTATION_CIRCLE;
  int ORIENTATION_DIST;
  float APPROACH_DIST;
  float HEIGHT_HIGH_THRES;
  float HEIGHT_LOW_THRES;
};

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_POSE_FINDER_H_

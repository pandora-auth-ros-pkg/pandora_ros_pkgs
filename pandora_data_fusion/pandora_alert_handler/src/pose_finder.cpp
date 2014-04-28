// "Copyright [year] <Copyright Owner>"

#include "alert_handler/pose_finder.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

PoseFinder::PoseFinder(const MapPtr& map, const std::string& mapType) 
  : map_(map)
{
  listener_.reset( TfFinder::newTfListener(mapType) );
}

void PoseFinder::updateParams(float occupiedCellThres,
      float heightHighThres, float heightLowThres,
      float approachDist, float orientationDist, float orientationCircle)
{
  OCCUPIED_CELL_THRES = occupiedCellThres;
  HEIGHT_HIGH_THRES = heightHighThres;
  HEIGHT_LOW_THRES = heightLowThres;
  ORIENTATION_CIRCLE = orientationCircle;
  ORIENTATION_DIST = orientationDist;
  APPROACH_DIST = approachDist;
}

void PoseFinder::publishVisionTransform(float alertYaw, float alertPitch, 
    tf::Transform worldHeadCameraTransform)
{
  tfScalar cameraYaw, cameraPitch, cameraRoll;

  worldHeadCameraTransform.getBasis().getRPY(cameraRoll,
                                             cameraPitch, cameraYaw);

  tf::Transform transVision = worldHeadCameraTransform;
  tf::Quaternion rotation;
  rotation.setRPY(0, cameraPitch - alertPitch, cameraYaw - alertYaw);
  transVision.setRotation(rotation);

  // victimFrameBroadcaster.sendTransform(
    // tf::StampedTransform( transVision , ros::Time::now(), "world", "vision")  );
}

Pose PoseFinder::findAlertPose(float alertYaw, float alertPitch,
    tf::Transform tfTransform)
{
  Pose outPose;

  publishVisionTransform(alertYaw, alertPitch, tfTransform);

  tfScalar pitch, roll, yaw, horizontalDirection, verticalDirection;

  tfTransform.getBasis().getRPY(roll, pitch, yaw);
  tf::Vector3 origin = tfTransform.getOrigin();
  
  horizontalDirection = yaw - alertYaw;
  verticalDirection = alertPitch - pitch;

  Point framePosition = Utils::vector3ToPoint(origin);

  Point position = positionOnWall(framePosition, horizontalDirection);

  float distFromAlert = Utils::distanceBetweenPoints2D(
                            position, framePosition);

  float height = calcHeight(verticalDirection, framePosition.z, distFromAlert);

  outPose.position = Utils::point2DAndHeight2Point3D(position, height);
  outPose.orientation = findNormalVectorOnWall(framePosition, outPose.position);

  return outPose;
}

float PoseFinder::calcHeight(float alertPitch,
    float height, float distFromAlert)
{
  float alertHeight = tan(alertPitch) * distFromAlert;

  alertHeight += height;

  ROS_DEBUG_NAMED("pose_finder",
    "[ALERT_HANDLER]Height of alert = %f ", alertHeight);
  ROS_DEBUG_NAMED("pose_finder", "[ALERT_HANDLER]Distance from alert = %f ",
      distFromAlert);

  if (alertHeight > HEIGHT_HIGH_THRES || alertHeight < HEIGHT_LOW_THRES)
    throw AlertException("Alert either too low or two high");

  return alertHeight;
}

Point PoseFinder::positionOnWall(Point startPoint, float angle)
{
  const float resolution = map_->info.resolution;
  float x = 0, y = 0, D = 5 * resolution;

  float currX = startPoint.x;
  float currY = startPoint.y;

  float omega = angle;

  x = D * cos(omega) + currX;
  y = D * sin(omega) + currY;

  while (map_->data[COORDS(x, y, map_)]
      < OCCUPIED_CELL_THRES * 100)
  {
    D += resolution;
    x = D * cos(omega) + currX;
    y = D * sin(omega) + currY;
  }
  if (map_->data[COORDS(x, y, map_)]
      > OCCUPIED_CELL_THRES * 100)
  {
    Point onWall;
    onWall.x = x;
    onWall.y = y;
    return onWall;
  } 
  else 
    throw AlertException("Can not find point on wall");
}

geometry_msgs::Quaternion PoseFinder::findNormalVectorOnWall(Point framePoint,
    Point alertPoint)
{
  std::vector<Point> points;
  float x = 0, y = 0;

  for (unsigned int i = 0; i < 360; i += 5)
  {
    x = alertPoint.x + ORIENTATION_CIRCLE * cos((i / 180.0) * PI);
    y = alertPoint.y + ORIENTATION_CIRCLE * sin((i / 180.0) * PI);

    if (map_->data[COORDS(x, y, map_)]
        > OCCUPIED_CELL_THRES * 100)
    {
      Point temp;
      temp.x = x;
      temp.y = y;
      points.push_back(temp);
    }
  }

  std::pair<Point, Point> pointsOnWall =
                                   findDiameterEndPointsOnWall(points);

  float angle;

  //!< if points are too close, first point should be the
  //!< diametrically opposite of the second
  if ( Utils::distanceBetweenPoints2D
      (pointsOnWall.first, pointsOnWall.second) < ORIENTATION_CIRCLE / 2 )
  {  
    angle = atan2((alertPoint.y - pointsOnWall.second.y),
        (alertPoint.x - pointsOnWall.second.x));
    
    Point onWall;
    onWall.x = alertPoint.x + ORIENTATION_CIRCLE * cos(angle);
    onWall.y = alertPoint.y + ORIENTATION_CIRCLE * sin(angle);
    pointsOnWall.first = onWall;
  }

  angle = atan2((pointsOnWall.second.y - pointsOnWall.first.y), 
      (pointsOnWall.second.x - pointsOnWall.first.x));

  std::pair<Point, Point> approachPoints;

  Point first;
  first.x = alertPoint.x + ORIENTATION_DIST * cos((PI / 2) + angle);
  first.y = alertPoint.y + ORIENTATION_DIST * sin((PI / 2) + angle);
  approachPoints.first = first;

  Point second;
  second.x = alertPoint.x + ORIENTATION_DIST * cos((-PI / 2) + angle);
  second.y = alertPoint.y + ORIENTATION_DIST * sin((-PI / 2) + angle);
  approachPoints.second = second;

  if ( Utils::distanceBetweenPoints2D(framePoint, approachPoints.first) < 
        Utils::distanceBetweenPoints2D(framePoint, approachPoints.second) )
  {
    return Utils::calculateQuaternion(alertPoint, approachPoints.first);
  }
  else
  {
    return Utils::calculateQuaternion(alertPoint, approachPoints.second);
  }
}

std::pair<Point, Point> PoseFinder::findDiameterEndPointsOnWall(
    std::vector<Point> points)
{
  if (points.size() < 2)
  {
    throw AlertException("Can not calculate approach point");
  }

  float maxDist = 0, dist = 0;

  std::pair<Point, Point> pointsOnWall;

  for (unsigned int i = 0; i < points.size(); i++)
  {
    for (unsigned int j = i + 1; j < points.size(); j++)
    {
      dist = Utils::distanceBetweenPoints2D(points[i], points[j]);
      if (dist > maxDist)
      {
        maxDist = dist;
        pointsOnWall = std::make_pair(points[i], points[j]);
      }
    }
  }

  return pointsOnWall;
}


tf::Transform PoseFinder::lookupTransformFromWorld(std_msgs::Header header)
{
  tf::StampedTransform tfTransform;

  listener_->waitForTransform("/map", header.frame_id,
      header.stamp, ros::Duration(1));

  listener_->lookupTransform( "/map", header.frame_id, 
      header.stamp, tfTransform);

  return tfTransform;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


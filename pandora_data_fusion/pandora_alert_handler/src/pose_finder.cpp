// "Copyright [year] <Copyright Owner>"

#include <utility> 
#include <vector> 

#include "alert_handler/pose_finder.h"

PoseFinder::PoseFinder(const Map& map, float heightHighThres,
    float heightLowThres,
    float approachDist, int orientationDist,
    int orientationCircle) : _map(map) {

  updateParams(heightHighThres, heightLowThres, approachDist,
                            orientationDist, orientationCircle);

  tf::StampedTransform tfTransform;

  try {
    _listener.waitForTransform("/world", "/map",
                             ros::Time(0), ros::Duration(1));
    _listener.lookupTransform("/world", "/map", ros::Time(0), tfTransform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("[ALERT_HANDLER %d]%s", __LINE__, ex.what());
  }

}

void PoseFinder::updateParams(float heightHighThres, float heightLowThres,
      float approachDist, int orientationDist, int orientationCircle) {

  HEIGHT_HIGH_THRES = heightHighThres;
  HEIGHT_LOW_THRES = heightLowThres;
  ORIENTATION_CIRCLE = orientationCircle;
  ORIENTATION_DIST = orientationDist;
  APPROACH_DIST = approachDist;
}

void PoseFinder::publishVisionTransform(float alertYaw, float alertPitch,
                                      tf::Transform worldHeadCameraTransform) {

  tfScalar cameraYaw, cameraPitch, cameraRoll;

  worldHeadCameraTransform.getBasis().getRPY(cameraRoll,
                                             cameraPitch, cameraYaw);

  tf::Transform transVision = worldHeadCameraTransform;
  tf::Quaternion rotation;
  rotation.setRPY(0, cameraPitch - alertPitch, cameraYaw - alertYaw);
  transVision.setRotation(rotation);

  victimFrameBroadcaster.sendTransform(
    tf::StampedTransform( transVision , ros::Time::now(), "world", "vision")  );

}


Pose PoseFinder::findAlertPose(float alertYaw, float alertPitch,
    tf::Transform tfTransform) {
  Pose outPose;

  publishVisionTransform(alertYaw, alertPitch, tfTransform);

  tfScalar pitch, roll, yaw, horizontalDirection, verticalDirection;

  tfTransform.getBasis().getRPY(roll, pitch, yaw);
  tf::Vector3 origin = tfTransform.getOrigin();

  horizontalDirection = yaw - alertYaw;
  verticalDirection = alertPitch - pitch;

  Point framePosition = Utils::vector3ToPoint(origin);

  PixelCoords position = positionOnWall(framePosition, horizontalDirection);

  float distFromAlert = Utils::distanceBetweenPoints2D(
                            Utils::pixelCoordsToPoint(position), framePosition);

  float height = calcHeight(verticalDirection, origin[2], distFromAlert);

  outPose.position = Utils::pixelCoordsAndHeight2Point(position, height);
  outPose.orientation = findNormalVectorOnWall(framePosition, outPose.position);

  return outPose;

}

float PoseFinder::calcHeight(float alertPitch,
    float height, float distFromAlert) {
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


PixelCoords PoseFinder::positionOnWall(Point startPoint, float angle) {
  int x = 0, y = 0, D = 5;
  PixelCoords onWall;
  PixelCoords startCoords = Utils::pointToPixelCoords(startPoint);

  unsigned int currX = startCoords.getXCoord();
  unsigned int currY = startCoords.getYCoord();

  float omega = angle;

  x = D * cos(omega) + currX;
  y = D * sin(omega) + currY;

  while (_map[x][y] > 127) {
    D++;
    x = D * cos(omega) + currX;
    y = D * sin(omega) + currY;

  }
  if (_map[x][y] == 127) {
    throw AlertException("Can not find point on wall");
  }
  if (_map[x][y] < 127) {
    onWall = PixelCoords(x, y);
    return onWall;
  }
  throw AlertException("Can not find point on wall");
}

geometry_msgs::Quaternion PoseFinder::findNormalVectorOnWall(Point framePoint,
    Point alertPoint) {
  PixelCoords frameCoords = Utils::pointToPixelCoords(framePoint);
  PixelCoords alertCoords = Utils::pointToPixelCoords(alertPoint);

  std::vector<PixelCoords> points;
  int x, y;

  for (unsigned int i = 0; i < 360; i += 5) {
    x = alertCoords.getXCoord() + ORIENTATION_CIRCLE * cos((i / 180.0) * D_PI);
    y = alertCoords.getYCoord() + ORIENTATION_CIRCLE * sin((i / 180.0) * D_PI);

    if (_map[x][y] < 127) {
      points.push_back(PixelCoords(x, y));
    }
  }

  std::pair<PixelCoords, PixelCoords> pointsOnWall =
                                   findDiameterEndPointsOnWall(points);

  float angle;

  // if points are too close, first point should be the
  // diametrically opposite of the second
  if (pointsOnWall.first.computeDistanceFrom(pointsOnWall.second) < 
                                                      ORIENTATION_CIRCLE / 2) {
    
    angle = atan2((alertCoords.getYCoord() - pointsOnWall.first.getYCoord()),
      (alertCoords.getXCoord() - pointsOnWall.first.getXCoord()));
      
    pointsOnWall.first = PixelCoords(alertCoords.getXCoord() -
      ORIENTATION_CIRCLE * cos(D_PI + angle), alertCoords.getYCoord() - 
        ORIENTATION_CIRCLE * sin(D_PI + angle));
  }

  angle = atan2((pointsOnWall.second.getYCoord() - 
    pointsOnWall.first.getYCoord()), (pointsOnWall.second.getXCoord() - 
      pointsOnWall.first.getXCoord()));

  std::pair<PixelCoords, PixelCoords> approachPoints;

  approachPoints.first = PixelCoords(alertCoords.getXCoord() + 
    ORIENTATION_DIST * cos((D_PI / 2) + angle), alertCoords.getYCoord() + 
      ORIENTATION_DIST * sin((D_PI / 2) + angle));

  approachPoints.second = PixelCoords(alertCoords.getXCoord() + 
    ORIENTATION_DIST * cos((-D_PI / 2) + angle), alertCoords.getYCoord() + 
      ORIENTATION_DIST * sin((-D_PI / 2) + angle));

  if (frameCoords.computeDistanceFrom(approachPoints.first) < 
                      frameCoords.computeDistanceFrom(approachPoints.second)) {
    return Utils::calculateQuaternion(alertCoords, approachPoints.first);
  } else {
    return Utils::calculateQuaternion(alertCoords, approachPoints.second);
  }
}


std::pair<PixelCoords, PixelCoords> PoseFinder::findDiameterEndPointsOnWall(
    std::vector<PixelCoords> points) {
  if (points.size() < 2) {
    throw AlertException("Can not calculate approach point");
  }

  float maxDist = 0, dist = 0;

  std::pair<PixelCoords, PixelCoords> pointsOnWall;

  for (unsigned int i = 0; i < points.size(); i++) {
    for (unsigned int j = i + 1; j < points.size(); j++) {
      dist = points[i].computeDistanceFrom(points[j]);
      if (dist > maxDist) {
        maxDist = dist;
        pointsOnWall = std::make_pair(points[i], points[j]);
      }
    }
  }

  return pointsOnWall;
}


tf::Transform PoseFinder::lookupTransformFromWorld(std_msgs::Header header) {
  tf::StampedTransform tfTransform;

  try {

    _listener.waitForTransform("/world", header.frame_id,
                                     header.stamp, ros::Duration(1));

    _listener.lookupTransform( "/world", header.frame_id,
                                           header.stamp, tfTransform);

  } catch (tf::TransformException ex) {
    ROS_ERROR("[ALERT_HANDLER %d]%s", __LINE__, ex.what());
    throw AlertException(
        "Something went wrong with tf, ignoring current message");
  }

  return tfTransform;
}


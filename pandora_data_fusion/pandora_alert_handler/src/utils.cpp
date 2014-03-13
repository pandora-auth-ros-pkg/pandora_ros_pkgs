// "Copyright [year] <Copyright Owner>"

#include "alert_handler/utils.h"

float Utils::pixelCoords2Meters(int pixels) {
  return pixels * OCGD;
}

int Utils::meters2PixelCoords(float meters) {
  return floor(meters / OCGD);
}

PixelCoords Utils::pointToPixelCoords(Point point) {
  PixelCoords pixels(meters2PixelCoords(point.x) +  
    START_X, meters2PixelCoords(point.y) + START_Y);
  return pixels;
}

Point Utils::pixelCoordsToPoint(PixelCoords pixels) {
  Point point;
  point.x = pixelCoords2Meters(pixels.getXCoord() - START_X);
  point.y = pixelCoords2Meters(pixels.getYCoord() - START_Y);
  return point;
}

Point Utils::pixelCoordsAndHeight2Point(PixelCoords position, float height) {
  Point point;
  point = pixelCoordsToPoint(position);
  point.z = height;
  return point;
}

float Utils::distanceBetweenPoints2D(Point a, Point b) {
  float xDist = a.x - b.x;
  float yDist = a.y - b.y;

  return sqrt((xDist * xDist) + (yDist * yDist));
}

float Utils::distanceBetweenPoints3D(Point a, Point b) {
  float xDist = a.x - b.x;
  float yDist = a.y - b.y;
  float zDist = a.z - b.z;

  return sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));
}


geometry_msgs::Quaternion Utils::calculateQuaternion(PixelCoords a,
    PixelCoords b) {
  tfScalar yaw;

  yaw = atan2(b.getYCoord() - a.getYCoord(), b.getXCoord() - a.getXCoord());

  return tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
}


Point Utils::vector3ToPoint(tf::Vector3 vector) {
  Point point;
  point.x = vector[0];
  point.y = vector[1];
  point.z = vector[2];

  return point;
}


bool Utils::arePointsInRange(Point pointA, Point pointB, float sensor_range ) {
  float dist = distanceBetweenPoints2D(pointA, pointB);

  if (dist > sensor_range)
    return false;
  else
    return true;
}

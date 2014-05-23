// "Copyright [year] <Copyright Owner>"

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    Point Utils::point2DAndHeight2Point3D(Point position, float height)
    {
      position.z = height;
      return position;
    }

    float Utils::distanceBetweenPoints2D(Point a, Point b)
    {
      float xDist = a.x - b.x;
      float yDist = a.y - b.y;

      return sqrt((xDist * xDist) + (yDist * yDist));
    }

    float Utils::distanceBetweenPoints3D(Point a, Point b)
    {
      float xDist = a.x - b.x;
      float yDist = a.y - b.y;
      float zDist = a.z - b.z;

      return sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));
    }

    geometry_msgs::Quaternion Utils::calculateQuaternion(Point a, 
        Point b)
    {
      tfScalar yaw;

      yaw = atan2(b.y - a.y, b.x - a.x);

      return tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    }

    Point Utils::vector3ToPoint(tf::Vector3 vector)
    {
      Point point;
      point.x = vector[0];
      point.y = vector[1];
      point.z = vector[2];

      return point;
    }

    bool Utils::arePointsInRange(Point pointA, Point pointB, float sensor_range )
    {
      float dist = distanceBetweenPoints2D(pointA, pointB);

      if (dist > sensor_range)
        return false;
      else
        return true;
    }

    float Utils::probabilityFromStdDev(float boundingRadius, float deviation)
    {
      if(boundingRadius <= 0)
      {
        throw std::range_error("Bounding radius haw always a positive value.");
      }
      if(deviation < 0)
      {
        throw std::range_error("Standard deviation is a positive value.");
      }
      if(deviation == 0)
        return 1;
      float x = boundingRadius / deviation;
      return 1 - exp(-pow(x, 2) / 2);
    }

    float Utils::stdDevFromProbability(float boundingRadius, float probability)
    {
      if(boundingRadius <= 0)
      {
        throw std::range_error("Bounding radius haw always a positive value.");
      }
      if(probability > 1 || probability < 0)
      {
        throw std::range_error("Probability value is between 0 and 1.");
      }
      return boundingRadius / sqrt(log(pow(1 - probability, -2)));
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion


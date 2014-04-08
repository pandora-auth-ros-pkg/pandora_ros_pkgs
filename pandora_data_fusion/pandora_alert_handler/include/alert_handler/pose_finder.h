// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_POSE_FINDER_H
#define ALERT_HANDLER_POSE_FINDER_H

#include <utility> 
#include <vector>
#include <boost/utility.hpp>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/OccupancyGrid.h>

#include "alert_handler/defines.h"
#include "alert_handler/tf_finder.h"
#include "alert_handler/tf_listener.h"
#include "alert_handler/objects.h"
#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class PoseFinder : private boost::noncopyable
{
 public:

  PoseFinder(const MapPtr& map, const std::string& mapType,
    float occupiedCellThres = 0.5,
    float heightHighThres = 1.2, float heightLowThres = 0,
    float approachDist = 0.5, float orientationDist = 0.5,
    float orientationCircle = 0.25);
  Pose findAlertPose(float alertYaw, float alertPitch,
    tf::Transform tfTransform);
  tf::Transform lookupTransformFromWorld(std_msgs::Header header);

  void updateParams(float occupiedCellThres,
    float heightHighThres, float heightLowThres,
    float approachDist,
    float orientationDist, float orientationCircle);

 private:

  Point positionOnWall(Point startPoint, float angle);
  float calcHeight(float alertPitch, float height, float distFromAlert);
  geometry_msgs::Quaternion findNormalVectorOnWall(Point framePoint,
      Point alertPoint);
  std::pair<Point, Point> findDiameterEndPointsOnWall(
      std::vector<Point> points);

  void publishVisionTransform(float alertYaw, float alertPitch,
      tf::Transform worldHeadCameraTransform);

 private:

  friend class PoseFinderTest;

 private:

  const MapPtr& map_;

  TfListenerPtr listener_;
  // tf::TransformBroadcaster victimFrameBroadcaster;

  //!< params
  float ORIENTATION_CIRCLE;
  float ORIENTATION_DIST;
  float APPROACH_DIST;
  float HEIGHT_HIGH_THRES;
  float HEIGHT_LOW_THRES;
  float OCCUPIED_CELL_THRES;

};

typedef boost::scoped_ptr< PoseFinder > PoseFinderPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_POSE_FINDER_H

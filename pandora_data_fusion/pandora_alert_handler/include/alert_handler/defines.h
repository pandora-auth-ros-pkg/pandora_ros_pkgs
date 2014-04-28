// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_DEFINES_H
#define ALERT_HANDLER_DEFINES_H

#include <boost/math/constants/constants.hpp>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

//!< Set that boost::noncopyable will set private default constructors
#define BOOST_NO_DEFAULTED_FUNCTIONS

//!< Macro for pi.
#define PI boost::math::constants::pi<float>()

//!< Macro to convert to map coordinates from meters.
#define COORDS(X, Y, MAP) ceil((X - MAP->info.origin.position.x)\
    / MAP->info.resolution) + ceil((Y - MAP->info.origin.position.y)\
      / MAP->info.resolution) * MAP->info.width

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;

typedef std::vector<PoseStamped> PoseStampedVector;
typedef nav_msgs::OccupancyGrid Map;
typedef nav_msgs::OccupancyGridPtr MapPtr;
typedef nav_msgs::OccupancyGridConstPtr MapConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_DEFINES_H

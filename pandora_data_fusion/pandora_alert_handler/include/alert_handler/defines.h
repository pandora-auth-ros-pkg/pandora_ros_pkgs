/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef ALERT_HANDLER_DEFINES_H
#define ALERT_HANDLER_DEFINES_H

#include <vector>
#include <cmath>
#include <boost/math/constants/constants.hpp>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

//!< Set that boost::noncopyable will set private default constructors
#ifndef BOOST_NO_DEFAULTED_FUNCTIONS
#define BOOST_NO_DEFAULTED_FUNCTIONS
#endif

//!< Macro for pi.
#define PI boost::math::constants::pi<float>()

//!< Macro for a degree in radians.
#define DEGREE (PI / 180.0)

//!< Macro to convert to map coordinates from meters.
#define COORDS(X, Y, MAP) static_cast<int>(floor((X - MAP->info.origin.position.x)\
    / MAP->info.resolution) + floor((Y - MAP->info.origin.position.y)\
      / MAP->info.resolution) * MAP->info.width)

//!< Macro to convert to map coordinates from meters.
#define CELL(X, Y, MAP) MAP->data[COORDS(X, Y, MAP)]

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

    typedef std::vector<geometry_msgs::PoseStamped> PoseStampedVector;
    typedef nav_msgs::OccupancyGrid Map;
    typedef nav_msgs::OccupancyGridPtr MapPtr;
    typedef nav_msgs::OccupancyGridConstPtr MapConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_DEFINES_H

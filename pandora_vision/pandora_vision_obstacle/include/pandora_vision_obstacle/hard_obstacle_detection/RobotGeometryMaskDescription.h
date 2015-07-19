/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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

#ifndef PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_ROBOTGEOMETRYMASKDESCRIPTION_H
#define PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_ROBOTGEOMETRYMASKDESCRIPTION_H

#include "opencv2/core/core.hpp"
#include <boost/shared_ptr.hpp>

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  /**
   * @class RobotGeometryMaskDescription TODO
   */
  class RobotGeometryMaskDescription
  {
   public:
    typedef boost::shared_ptr<RobotGeometryMaskDescription> Ptr;
    typedef boost::shared_ptr<RobotGeometryMaskDescription const> ConstPtr;

   public:
    RobotGeometryMaskDescription()
    {
    }
    virtual ~RobotGeometryMaskDescription()
    {
    }

    // cv::Mat mask;  // square mask simulating /base_footprint in dim totalD x totalD

    double wheelH;  // height measured at 0cm from /base_footprint
    double barrelH;  // height measured at 6cm from /base_footprint
    double robotH;  // height measured at 12cm from /base_footprint

    double wheelD;  // width of wheel foot - measured 7cm
    double barrelD;  // width of motor barrel - measured 10cm
    double robotD;  // width of main robot - measured 8cm
    // half robot mask (al,ar,ah,ab,bl,br,bt,bb) is in max dim equal to totalD
    // and in min dim equal to (wheelD + barrelD + robotD/2)
    double totalD;  // total width/height of square mask

    /// Tolerance for the difference used to determine if the robot can move to certain
    /// position without collisions.
    double eps;

    /// Max possible Angle for the robot's orientation in Degrees.
    double maxPossibleAngle;

    double RESOLUTION;
  };

  typedef RobotGeometryMaskDescription::Ptr RobotGeometryMaskDescriptionPtr;
  typedef RobotGeometryMaskDescription::ConstPtr RobotGeometryMaskDescriptionConstPtr;
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_ROBOTGEOMETRYMASKDESCRIPTION_H


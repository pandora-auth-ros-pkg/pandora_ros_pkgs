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

#ifndef PANDORA_VISION_COMMON_POI_H
#define PANDORA_VISION_COMMON_POI_H

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

namespace pandora_vision
{
  class POI
  {
   public:
    typedef boost::shared_ptr<POI> Ptr;
    typedef boost::shared_ptr<POI const> ConstPtr;

   public:
    /// Coordinates of Point of Interest
    cv::Point2f point;

    /// The possibility of this actually being a Point of Interest
    float probability;

   public:
    POI() {}
    virtual ~POI() {}

    void setPoint(const cv::Point&);
    cv::Point getPoint() const;

    void setProbability(float probArg);
    float getProbability() const;
  };

  void POI::setPoint(const cv::Point& pointArg)
  {
    point = pointArg;
  }

  cv::Point POI::getPoint() const
  {
    return point;
  }

  void POI::setProbability(float probArg)
  {
    probability = probArg;
  }

  float POI::getProbability() const
  {
    return probability;
  }

  typedef POI::Ptr POIPtr;
  typedef POI::ConstPtr POIConstPtr;
}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_POI_H

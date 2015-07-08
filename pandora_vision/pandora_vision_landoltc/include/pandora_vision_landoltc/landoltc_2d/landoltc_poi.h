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
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_LANDOLTC_LANDOLTC_2D_LANDOLTC_POI_H
#define PANDORA_VISION_LANDOLTC_LANDOLTC_2D_LANDOLTC_POI_H

#include <string>
#include <vector>

#include "pandora_vision_common/poi.h"

namespace pandora_vision
{
namespace pandora_vision_landoltc
{
  class LandoltCPOI : public POI
  {
    public:
      typedef boost::shared_ptr<LandoltCPOI> Ptr;

      virtual ~LandoltCPOI() {}

    public:
      std::vector<float> angles;
      std::vector<cv::Scalar> color;
      std::vector<cv::Rect> bbox;
      int count;

    public:
      void setAngles(const std::vector<float>& angles)
      {
        this->angles = angles;
      }
      void setColor(const std::vector<cv::Scalar>& color)
      {
        this->color = color;
      }
      void setBox(const std::vector<cv::Rect>& box)
      {
        bbox = box;
      }
      void setCount(const int& count)
      {
        this->count = count;
      }

      std::vector<float> getAngles() const
      {
        return angles;
      }

      std::vector<cv::Scalar> getColorVector() const
      {
        return color;
      }
      cv::Scalar getColor(int pos) const
      {
        return color[pos];
      }

      std::vector<cv::Rect> getBoxVector() const
      {
        return bbox;
      }
      cv::Rect getBox(int pos) const
      {
        return bbox[pos];
      }

      int getCount() const
      {
        return count;
      }
  };

  typedef LandoltCPOI::Ptr LandoltCPOIPtr;
}  // namespace pandora_vision_landoltc
}  // namespace pandora_vision

#endif  // PANDORA_VISION_LANDOLTC_LANDOLTC_2D_LANDOLTC_POI_H

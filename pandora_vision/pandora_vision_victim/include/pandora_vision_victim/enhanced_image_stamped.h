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
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_VICTIM_ENHANCED_IMAGE_STAMPED_H
#define PANDORA_VISION_VICTIM_ENHANCED_IMAGE_STAMPED_H

#include <vector>

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include "pandora_vision_victim/images_stamped.h"

namespace pandora_vision
{
  class EnhancedImageStamped : public ImagesStamped
  {
    public:
      typedef boost::shared_ptr<EnhancedImageStamped> Ptr;
      typedef boost::shared_ptr<EnhancedImageStamped const> ConstPtr;
      typedef cv::Rect_<float> Rect2f;

    public:
      bool isDepth;
      std::vector<Rect2f> regionsOfInterest;

    public:
      void setDepth(bool depth);
      bool getDepth() const;

      void setRegions(const std::vector<Rect2f>&);
      std::vector<Rect2f> getRegions() const;

      void setRegion(int , const Rect2f&);
      Rect2f getRegion(int it) const;
  };

  void EnhancedImageStamped::setDepth(bool depth)
  {
    isDepth = depth;
  }

  bool EnhancedImageStamped::getDepth() const
  {
    return isDepth;
  }

  void EnhancedImageStamped::setRegions(const std::vector<EnhancedImageStamped::Rect2f>& regions)
  {
    regionsOfInterest = regions;
  }

  std::vector<EnhancedImageStamped::Rect2f> EnhancedImageStamped::getRegions() const
  {
    return regionsOfInterest;
  }

  void EnhancedImageStamped::setRegion(int it, const Rect2f& region)
  {
    if (it == 0)
    {
      regionsOfInterest.clear();
    }
    regionsOfInterest.push_back(region);
  }

  EnhancedImageStamped::Rect2f EnhancedImageStamped::getRegion(int it) const
  {
    return regionsOfInterest[it];
  }

  typedef EnhancedImageStamped::Rect2f Rect2f;
  typedef EnhancedImageStamped::Ptr EnhancedImageStampedPtr;
  typedef EnhancedImageStamped::ConstPtr EnhancedImageStampedConstPtr;
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_ENHANCED_IMAGE_STAMPED_H

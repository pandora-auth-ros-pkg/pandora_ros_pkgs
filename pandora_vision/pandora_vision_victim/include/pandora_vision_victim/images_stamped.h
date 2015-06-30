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

#ifndef PANDORA_VISION_VICTIM_IMAGES_STAMPED_H
#define PANDORA_VISION_VICTIM_IMAGES_STAMPED_H

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Header.h"

namespace pandora_vision
{
  class ImagesStamped
  {
    public:
      typedef boost::shared_ptr<ImagesStamped> Ptr;
      typedef boost::shared_ptr<ImagesStamped const> ConstPtr;

    public:
      std_msgs::Header header;

      cv::Mat depthImage;
      cv::Mat rgbImage;

    public:
      void setHeader(const std_msgs::Header&);
      const std_msgs::Header& getHeader() const;

      void setRgbImage(const cv::Mat&);
      cv::Mat getRgbImage() const;

      void setDepthImage(const cv::Mat&);
      cv::Mat getDepthImage() const;
  };

  void ImagesStamped::setHeader(const std_msgs::Header& headerArg)
  {
    header = headerArg;
  }

  const std_msgs::Header& ImagesStamped::getHeader() const
  {
    return header;
  }

  void ImagesStamped::setRgbImage(const cv::Mat& imageArg)
  {
    rgbImage = imageArg;
  }

  cv::Mat ImagesStamped::getRgbImage() const
  {
    return rgbImage;
  }

  void ImagesStamped::setDepthImage(const cv::Mat& imageArg)
  {
    depthImage = imageArg;
  }

  cv::Mat ImagesStamped::getDepthImage() const
  {
    return depthImage;
  }

  typedef ImagesStamped::Ptr ImagesStampedPtr;
  typedef ImagesStamped::ConstPtr ImagesStampedConstPtr;
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_IMAGES_STAMPED_H

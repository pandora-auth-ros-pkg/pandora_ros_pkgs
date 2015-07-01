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

#ifndef PANDORA_VISION_COMMON_POIS_STAMPED_H
#define PANDORA_VISION_COMMON_POIS_STAMPED_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <std_msgs/Header.h>

#include "pandora_vision_common/poi.h"

namespace pandora_vision
{
  class POIsStamped
  {
   public:
    typedef boost::shared_ptr<POIsStamped> Ptr;
    typedef boost::shared_ptr<POIsStamped const> ConstPtr;

   public:
    POIsStamped() {}
    virtual ~POIsStamped() {}

   public:
    /// Message Header referring to the Points of Interest in a frame
    std_msgs::Header header;

    /// Vector containing shared pointers to Point of Interest structure
    std::vector<POIPtr> pois;

    /// Frame's Width
    int frameWidth;

    /// Frame's Height
    int frameHeight;
  };

  typedef POIsStamped::Ptr POIsStampedPtr;
  typedef POIsStamped::ConstPtr POIsStampedConstPtr;
}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_POIS_STAMPED_H

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
 * Authors: Alexandros Filotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_DETECTOR_INCLUDE_RGB_NODE_DUMMY_RGB_H_
#define PANDORA_VISION_HOLE_DETECTOR_INCLUDE_RGB_NODE_DUMMY_RGB_H_

#include "utils/defines.h"
#include "utils/holes_conveyor.h"
#include "utils/message_conversions.h"
#include "vision_communications/CandidateHolesVectorMsg.h"

namespace pandora_vision
{
  class Rgb
  {
    private:

      //!< The ROS node handle
      ros::NodeHandle nodeHandle_;

      //!< The ROS subscriber for acquisition of the RGB image through the
      //depth sensor
      ros::Subscriber rgbImageSubscriber_;

      //!< The ROS publisher ofcandidate holes
      ros::Publisher rgbCandidateHolesPublisher_;

      /**
        @brief Callback for the rgbImage acquired through the
        rgb_depth_synchronizer node
        @param[in] rgbImage [const sensor_msgs::Image&] The RGB image
        @return void
       **/
      void rgbImageCallback(const sensor_msgs::Image& inImage);


    public:

      //!< Constructor
      Rgb(void);

      //!< Destructor
      ~Rgb(void);
  };

} // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_DETECTOR_INCLUDE_RGB_NODE_DUMMY_RGB_H_

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
 * Author: Despoina Paschalidou
 *********************************************************************/

#ifndef RGB_NODE_RGB_H
#define RGB_NODE_RGB_H

#include "utils/defines.h"
#include "vision_communications/CandidateHolesVectorMsg.h"
#include "state_manager/state_client.h"
#include "utils/parameters.h"
#include "utils/message_conversions.h"
#include "utils/wavelets.h"
#include "rgb_node/hole_detector.h"

namespace pandora_vision
{
  class Rgb
  {
    private:

      //!< The NodeHandle
      ros::NodeHandle _nh;

      //!< Frame processed by FaceDetector
      cv::Mat _holeFrame;

      //!<FaceDetector frame timestamp
      ros::Time _holeFrameTimestamp;

      //!< The ROS subscriber for acquisition of the RGB image through the
      //depth sensor
      ros::Subscriber _frameSubscriber;

      //!< The ROS publisher ofcandidate holes
      ros::Publisher rgbCandidateHolesPublisher_;

      //!< Current state of robot
      int curState;
      //!< Previous state of robot
      int prevState;

      //!< Variable used for State Managing
      bool holeNowON;

      //!< Class HoleDetector instance that finds and locates tha position
      //!< potentional holes in current frame
      HoleDetector _holeDetector;

      //!< The dynamic reconfigure (RGB) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::rgb_cfgConfig>
        server;

      //!< The dynamic reconfigure (RGB) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole_detector::rgb_cfgConfig>::
        CallbackType f;

      /**
        Function called when new ROS message appears, for front camera
        @param msg [const sensor_msgs::ImageConstPtr&] The message
        @return void
       **/
      void inputRgbImageCallback(const sensor_msgs::Image& inImage);

      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_hole_detector::rgb_cfgConfig&]
        @param[in] level [const uint32_t] The level (?)
        @return void
       **/
      void parametersCallback(
        const pandora_vision_hole_detector::rgb_cfgConfig& config,
        const uint32_t& level);


    public:

      //!< The constructor
      Rgb();

      //!< The destructor
      ~Rgb();
  };

} //namespace pandora_vision

#endif  // RGB_NODE_RGB_H

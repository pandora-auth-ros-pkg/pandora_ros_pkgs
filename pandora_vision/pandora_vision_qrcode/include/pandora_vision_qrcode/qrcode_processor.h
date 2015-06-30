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
 *   Miltiadis-Alexios Papadopoulos
 *   Vassilis Choutas <vasilis4ch@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_QRCODE_QRCODE_PROCESSOR_H
#define PANDORA_VISION_QRCODE_QRCODE_PROCESSOR_H

#include <vector>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "pandora_vision_common/pandora_vision_interface/vision_processor.h"
#include "pandora_vision_common/cv_mat_stamped.h"
#include "pandora_vision_common/pois_stamped.h"

#include "pandora_vision_qrcode/qrcode_detector.h"
#include "pandora_vision_qrcode/qrcode_cfgConfig.h"

namespace pandora_vision
{
  class QrCodeProcessor : public VisionProcessor
  {
    public:
      typedef boost::shared_ptr<QrCodePOI> QrCodePOIPtr;

      /**
       * @brief: The Main constructor for the QR code Processor objects.
       * @param ns[const std::string&]: The namespace for the node.
       * @param handler[sensor_processor::Handler*]: A pointer to the handler
       * of the processor used to access the nodehandle of the node
       */
      QrCodeProcessor(const std::string& ns,
          sensor_processor::Handler* handler);

      /**
       * @brief: The default constructor for the QR Code Processor objects
       */
      QrCodeProcessor();
      virtual ~QrCodeProcessor() {}

      virtual bool
        process(const CVMatStampedConstPtr& input,
            const POIsStampedPtr& output);

    private:
      boost::shared_ptr<QrCodeDetector> detectorPtr_;

      /// Debug image topic
      std::string debugTopic_;

      /// Debug images publisher flag
      bool debugPublish_;

      /// List of detected qrcodes
      std::vector<POIPtr> qrcode_list;

      /// Debug publisher for QrCodeDetector
      // boost::shared_ptr<image_transport::Publisher> debugPublisherPtr_;

      /// The dynamic reconfigure parameters' server
      dynamic_reconfigure::Server<pandora_vision_qrcode::qrcode_cfgConfig>
        server;
      /// The dynamic reconfigure parameters' callback
      dynamic_reconfigure::Server<pandora_vision_qrcode::qrcode_cfgConfig>
        ::CallbackType f;

      /**
       * @brief The function called when a parameter is changed
       * @param[in] config [const pandora_vision_qrcode::qrcode_cfgConfig&]
       * @param[in] level [const uint32_t] The level
       * @return void
       */
      void parametersCallback(
        const pandora_vision_qrcode::qrcode_cfgConfig& config,
        const uint32_t& level);
  };
}  // namespace pandora_vision
#endif  // PANDORA_VISION_QRCODE_QRCODE_PROCESSOR_H

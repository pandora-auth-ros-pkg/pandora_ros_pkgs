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

#include "pandora_vision_qrcode/qrcode_processor.h"

namespace pandora_vision
{

  /*
   * @brief The default constructor of the QR processor class
   */
  QrCodeProcessor::QrCodeProcessor() : VisionProcessor()
  {
  }

  /*
   * @brief: The Main constructor for the QR code Processor objects.
   * @param ns[const std::string&]: The namespace for the node.
   * @param handler[sensor_processor::Handler*]: A pointer to the handler
   * of the processor used to access the nodehandle of the node
   */
  QrCodeProcessor::QrCodeProcessor(const std::string& ns,
      sensor_processor::Handler* handler) :
    VisionProcessor(ns, handler)
  {
    ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
        this->accessProcessorNh()->getNamespace());

    bool debugQrCode;
    // Get the image debug view flag.
    if (!this->accessPublicNh()->getParam("debugQrCode", debugQrCode))
      debugQrCode = false;

    ROS_DEBUG_STREAM("debugQrCode : " << (debugQrCode ? "True": "False"));

    int gaussianSharpenBlur;
    //!< Get the buffer size parameter if available;
    if (!this->accessPublicNh()->getParam("qrCodeSharpenBlur",
          gaussianSharpenBlur))
      gaussianSharpenBlur = 5;

    ROS_DEBUG_STREAM("qrCodeSharpenBlur : " << gaussianSharpenBlur);

    double gaussianSharpenWeight;
    //!< Get the difference threshold parameter if available;
    if (!this->accessPublicNh()->getParam("qrCodeSharpenWeight",
          gaussianSharpenWeight))
      gaussianSharpenWeight = 0.8;

    ROS_DEBUG_STREAM("qrCodeSharpenWeight : " << gaussianSharpenWeight);

    // Initiliaze the QR code detector.
    detectorPtr_.reset(new QrCodeDetector(gaussianSharpenBlur,
        gaussianSharpenWeight, debugQrCode));

    //!< The dynamic reconfigure parameter's callback
    server.setCallback(boost::bind(&QrCodeProcessor::parametersCallback,
          this, _1, _2));

    // if (debugQrcode_)
    // {
      // !< Advertise topics for debugging if we are in debug mode
      // if (this->accessPublicNh()->getParam("debug_qrcode_topic", debugTopic_))
      // {
        // debugPublisherPtr_.reset(new image_transport::Publisher());
        // *debugPublisherPtr_ = image_transport::ImageTransport(
          // *this->accessProcessorNh()).advertise(debugTopic_, 1); 
      // }
      // else
      // {
        // ROS_WARN("Cannot find qrcode debug show topic");
      // }
    // }
  }
  
  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_qrcode::qrcode_cfgConfig&]
    @param[in] level [const uint32_t] The level 
    @return void
  **/
  void QrCodeProcessor::parametersCallback(
    const pandora_vision_qrcode::qrcode_cfgConfig& config,
    const uint32_t& level)
  {
    detectorPtr_->set_debug(config.debugEnable);

    detectorPtr_->setSharpenBlur(config.sharpenBlur);
    detectorPtr_->setSharpenWeight(config.sharpenWeight);
    // if (debugQrcode_)
    // {
      // debugPublisherPtr_.reset(new image_transport::Publisher());
      // *debugPublisherPtr_ = image_transport::ImageTransport(
          // *this->accessProcessorNh()).advertise(debugTopic_, 1); 
    // }
    // else
      // debugPublisherPtr_.reset();
  }

  /*
   * @brief: The main process function that will be used to detect the QR
   * patterns on the current frame
   * @param input[const CVMatStampedConstPtr&]: The input image.
   * @param output[const POIsStampedPtr&]: The output regions of interest.
   * @return: True if any patterns were detected, false otherwise.
  */
  bool QrCodeProcessor::process(const CVMatStampedConstPtr& input,
      const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getImage().cols;
    output->frameHeight = input->getImage().rows;
    
    output->pois = detectorPtr_->detectQrCode(input->getImage());

    // if (debugPublish_)
    // {
      // cv_bridge::CvImage qrcodeDebug;
      // qrcodeDebug.encoding = sensor_msgs::image_encodings::MONO8;
      // qrcodeDebug.image = detectorPtr_->getDebugImage();
      // debugPublisherPtr_->publish(qrcodeDebug.toImageMsg());
    // }

    if (output->pois.empty())
    {
      return false;
    }
    return true;
  }
}  // namespace pandora_vision

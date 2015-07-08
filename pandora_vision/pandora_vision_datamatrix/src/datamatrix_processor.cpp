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
* Author: Despoina Paschalidou, Vasilis Bosdelekidis
*********************************************************************/

#include "pandora_vision_datamatrix/datamatrix_processor.h"

namespace pandora_vision
{
namespace pandora_vision_datamatrix
{
  /**
   *@brief Constructor
   **/
  void
    DatamatrixProcessor::initialize(const std::string& ns, sensor_processor::Handler* handler)
    {
      VisionProcessor::initialize(ns, handler);

      img = NULL;
      dec = NULL;
      reg = NULL;
      msg = NULL;

#ifdef DEBUG_MODE
      std::string debugTopic;
      if (this->getProcessorNodeHandle().getParam("debug_topic", debugTopic))
        ROS_DEBUG_STREAM("debugTopic : " << debugTopic);
      else
      {
        ROS_WARN("Cannot find datamatrix debug show topic");
      }
      _datamatrixPublisher = image_transport::ImageTransport(
          this->getProcessorNodeHandle()).advertise(debugTopic, 1);
#endif

      detected_datamatrix.reset( new DataMatrixPOI );
      detected_datamatrix->setContent("");

      detectorPtr_.reset(new DatamatrixDetector());

      ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
          this->getProcessorNodeHandle().getNamespace());
    }

  /**
    @brief Constructor
   **/
  DatamatrixProcessor::DatamatrixProcessor() : VisionProcessor() {}

  /**
    @brief Destructor
    */
  DatamatrixProcessor::~DatamatrixProcessor()
  {
    //!< Deallocate memory
    dmtxMessageDestroy(&msg);
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&img);
    dmtxRegionDestroy(&reg);
    ROS_INFO("[%s] destroyed", this->getName().c_str());
  }

   /**
   * @brief
   **/
  bool DatamatrixProcessor::process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getImage().cols;
    output->frameHeight = input->getImage().rows;

    output->pois = detectorPtr_->detect_datamatrix(input->getImage());

    if (output->pois.empty())
    {
      return false;
    }
    return true;
  }
}  // namespace pandora_vision_datamatrix
}  // namespace pandora_vision

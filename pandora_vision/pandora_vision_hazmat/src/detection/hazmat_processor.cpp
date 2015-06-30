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
 * Authors: Choutas Vassilis
 *********************************************************************/

#include <string>

#include "pandora_vision_hazmat/detection/hazmat_processor.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    HazmatProcessor::HazmatProcessor(const std::string& ns,
      sensor_processor::Handler* handler) :
      VisionProcessor(ns, handler), dynamicReconfServer_(*(this->accessProcessorNh()))
    {
      ROS_INFO("[%s] Starting Hazmat Detection!", this->getName().c_str());
      ROS_INFO("[%s] processor nh namespace: %s", this->getName().c_str(),
          this->accessProcessorNh()->getNamespace().c_str());

      // Get the path of the current package.
      std::string packagePath = ros::package::getPath("pandora_vision_hazmat");

      PlanarObjectDetector::setFileName(packagePath);

      // Initiliaze the object detector
      std::string featureType;

      if (!this->accessProcessorNh()->getParam("feature_type", featureType))
      {
        ROS_ERROR("[%s] Could not get the feature type parameter!",
            this->getName().c_str());
        ROS_INFO("[%s] Initializing default detector!",
            this->getName().c_str());
        featureType = "SIFT";
      }

      this->accessProcessorNh()->param<bool>("visualization",
          visualizationFlag_, false);
      this->accessProcessorNh()->param<bool>("execution_timer",
          execTimerFlag_, false);
      this->accessProcessorNh()->param<bool>("debug_message",
          debugMsgFlag_, true);

      detector_ = factory_.createDetectorObject(featureType);

      // Check if the detector was initialized.
      if (detector_ == NULL)
      {
        ROS_FATAL("[%s] Could not create a detector object!",
            this->getName().c_str());
        ROS_FATAL("[%s] The node will now exit!",
            this->getName().c_str());
        ROS_BREAK();
      }
      ROS_INFO_NAMED(this->getName().c_str(),
          "Created the detector object!");

      dynamicReconfServer_.setCallback(boost::bind(
            &HazmatProcessor::dynamicReconfigCallback, this, _1, _2));
    }

    HazmatProcessor::HazmatProcessor() : VisionProcessor() {}

    HazmatProcessor::~HazmatProcessor()
    {
      ROS_INFO("[%s] Stopping Hazmat Detection!", this->getName().c_str());
      delete detector_;
    }

    void HazmatProcessor::dynamicReconfigCallback(
        const ::pandora_vision_hazmat::DisplayConfig& config, uint32_t level)
    {
      if (detector_ == NULL)
      {
        ROS_ERROR("[%s] No detector object created!", this->getName().c_str());
        return;
      }
      // Check if the features type has changed. If yes create a new detector.
      if (detector_->getFeaturesName().compare(config.Feature_Type) != 0)
      {
        delete detector_;
        ROS_INFO("[%s] : Create new %s detector",
            this->getName().c_str(),
            config.Feature_Type.c_str());
        detector_ = factory_.createDetectorObject(config.Feature_Type);
      }

      detector_->setDisplayResultsFlag(config.Display_Results);
      detector_->setFeatureTimerFlag(config.Feature_Timer);
      detector_->setMaskDisplayFlag(config.Mask_Display);
      this->execTimerFlag_ = config.Execution_Timer;
      this->debugMsgFlag_ = config.Debug_Messages;
      this->visualizationFlag_ = config.Visualization_Flag;
    }

    bool HazmatProcessor::process(const CVMatStampedConstPtr& input,
      const POIsStampedPtr& output)
    {
      output->header = input->getHeader();
      output->frameWidth = input->getImage().cols;
      output->frameHeight = input->getImage().rows;

      std::stringstream ss;
      const clock_t begin_time = clock();
      PlanarObjectDetector::setDims(input->getImage());

      ROS_DEBUG_NAMED(this->getName().c_str(), "Detecting...");

      bool found = detector_->detect(input->getImage(), &output->pois);

      ROS_DEBUG_NAMED(this->getName().c_str(), "Detection done!!");
      double execTime = (clock() - begin_time) /
        static_cast<double>(CLOCKS_PER_SEC);

      // Check if the debug message printing is enabled.
      if (found && debugMsgFlag_)
      {
        for (int i = 0 ; i < output->pois.size() ; i++)
        {
          boost::shared_ptr<HazmatPOI> hazmatPOIPtr(
            boost::dynamic_pointer_cast<HazmatPOI>(output->pois[i]));
          ROS_DEBUG_NAMED("detection", "[%s] : Found Hazmat : %d",
              this->getName().c_str(),
              hazmatPOIPtr->getPattern());
        }
        ROS_DEBUG_NAMED("detection", "[%s] : Number of Hazmats"
            " Found = %d", this->getName().c_str(),
            static_cast<int>(output->pois.size()));
      }

      if (execTimerFlag_)
        ROS_DEBUG_NAMED(this->getName().c_str(), "Detection Execution Time"
            " is : %f!", execTime);

      if (visualizationFlag_)
      {
        cv::imshow("Input Image", input->getImage());
        char key = cv::waitKey(10);
        if ( key == 27)
        {
          ROS_INFO("[%s]: Goodbye! \n", this->getName().c_str());
          ros::shutdown();
        }
      }

      return found;
    }
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision

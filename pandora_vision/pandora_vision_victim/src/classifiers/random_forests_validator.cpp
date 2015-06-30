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
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include <string>

#include <ros/console.h>

#include "pandora_vision_victim/classifiers/random_forests_validator.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @brief Constructor. Initializes Random Forests classifier parameters and loads
   * classifier model.
   * @param classifierPath [const std::string&] The path to the classifier
   * model.
   */
  RandomForestsValidator::RandomForestsValidator(const ros::NodeHandle& nh,
      const std::string& imageType,
      const std::string& classifierType)
      : AbstractValidator(nh, imageType, classifierType)
  {
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Creating " << imageType
        << " " << classifierType << " Validator instance");
    randomForestsValidator_.load(classifierPath_.c_str());

    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Initialized " << imageType_ << " "
        << classifierType_ << " Validator instance");
  }

  /**
   * @brief Default Destructor.
   */
  RandomForestsValidator::~RandomForestsValidator()
  {
  }

  /**
   * @brief Function that loads the trained classifier and makes a prediction
   * according to the feature vector given for each image
   * @return void
   */
  void RandomForestsValidator::predict(const cv::Mat& featuresMat,
      float* classLabel, float* probability)
  {
    *classLabel = randomForestsValidator_.predict(featuresMat);
    *probability = randomForestsValidator_.predict_prob(featuresMat);
  }
}  // namespace pandora_vision


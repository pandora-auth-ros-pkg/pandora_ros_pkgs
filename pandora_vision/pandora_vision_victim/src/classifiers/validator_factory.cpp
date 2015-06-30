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
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*   Vassilis Choutas <vasilis4ch@gmail.com>
*********************************************************************/

#include <string>

#include <boost/algorithm/string.hpp>

#include "pandora_vision_victim/classifiers/validator_factory.h"
#include "pandora_vision_victim/classifiers/svm_validator.h"
#include "pandora_vision_victim/classifiers/neural_network_validator.h"
#include "pandora_vision_victim/classifiers/random_forests_validator.h"

namespace pandora_vision
{
  /**
   * @brief The constructor for the factory used to produce the validator/classifier objects.
   * @param ns[const std::string&] The namespace for the node handles of the object
  */
  ValidatorFactory::ValidatorFactory(const std::string& ns): nh_(ns)
  {
    ROS_INFO("[Victim_Validator_Factory]: Creating Validator Factory!");
    ROS_INFO("[Victim_Validator_Factory]: Finished creating Validator Factory!");
  }

  /**
   * @brief Creates a validator object depending on the image type and the validator type
   * provided.
   * @param nh[const ros::NodeHandle&] The nodehandle used by the node that wants to create a classifier
   * object.
   * @param validatorType[const std::string&] The type of classifier that will be used.
   * @param imageType[const std::string&] The type of images used for the classifier, such as
   * RGB images.
   * @return AbstractValidator* The pointer to the created object that will be used for object
   * detection.
  */
  AbstractValidator* ValidatorFactory::createValidator(const ros::NodeHandle& nh,
      const std::string& validatorType,
      const std::string& imageType)
  {
    std::string ns = nh_.getNamespace();

    AbstractValidator* validatorPtr;

    std::cout << "In factory : " << validatorType << std::endl;
    if (boost::iequals(validatorType, "svm"))
    {
      validatorPtr = new SvmValidator(ns, imageType, validatorType);
    }
    else if (boost::iequals(validatorType, "ann"))
    {
      validatorPtr =  new NeuralNetworkValidator(ns, imageType, validatorType);
    }
    else if (boost::iequals(validatorType, "rf"))
    {
      validatorPtr =  new RandomForestsValidator(ns, imageType, validatorType);
    }
    else
    {
      ROS_ERROR("[Victim_Validator_Factory]: Invalid Validator Type!");
      ROS_ERROR("[Victim_Validator_Factory]: The node will now shutdown!");
      return NULL;
    }

    ROS_INFO("[Victim_Validator_Factory]: Created %s Validator for %s images",
        boost::to_upper_copy<std::string>(validatorType).c_str(),
        boost::to_upper_copy<std::string>(imageType).c_str());
    return validatorPtr;
  }

}  //  namespace pandora_vision

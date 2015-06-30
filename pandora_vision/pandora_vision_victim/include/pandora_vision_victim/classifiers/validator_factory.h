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

#ifndef PANDORA_VISION_VICTIM_CLASSIFIERS_VALIDATOR_FACTORY_H
#define PANDORA_VISION_VICTIM_CLASSIFIERS_VALIDATOR_FACTORY_H

#include <string>

#include "pandora_vision_victim/classifiers/abstract_validator.h"

namespace pandora_vision
{
  /*
   * @class ClassifierFactory
   * @brief The class used to produce the different classifier objects
   * during runtime
  */
  class ValidatorFactory
  {
    public:
      /**
       * @brief The constructor for the factory used to produce the validator/classifier objects.
       * @param ns[const std::string&] The namespace for the node handles of the object
       */
      explicit ValidatorFactory(const std::string& ns);
      ~ValidatorFactory()
      { }

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
      AbstractValidator* createValidator(const ros::NodeHandle& nh,
          const std::string& validatorType,
          const std::string& imageType);

    private:
      ros::NodeHandle nh_;
  };
}  //  namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_CLASSIFIERS_VALIDATOR_FACTORY_H

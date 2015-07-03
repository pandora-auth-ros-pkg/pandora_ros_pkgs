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

#ifndef SENSOR_PROCESSOR_GENERAL_PROCESSOR_H
#define SENSOR_PROCESSOR_GENERAL_PROCESSOR_H

#include <string>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>

#include "sensor_processor/abstract_processor.h"
#include "sensor_processor/handler.h"

namespace sensor_processor
{
  /**
   * @class GeneralProcessor TODO
   */
  class GeneralProcessor : public AbstractProcessor
  {
   public:
    GeneralProcessor(void) {}
    virtual
    ~GeneralProcessor() {}
    virtual void
    initialize(const std::string& ns, Handler* handler);

   protected:
    ros::NodeHandlePtr accessPublicNh();
    ros::NodeHandlePtr accessProcessorNh();
    std::string getName();

   protected:
    ros::NodeHandlePtr publicNh_;
    ros::NodeHandlePtr processorNh_;
    std::string name_;
  };

  void
  GeneralProcessor::initialize(const std::string& ns, Handler* handler)
  {
    this->processorNh_.reset( new ros::NodeHandle(ns) );
    this->publicNh_ = handler->shareNodeHandle();
    this->name_ = boost::to_upper_copy<std::string>(this->processorNh_->getNamespace());
  }

  ros::NodeHandlePtr GeneralProcessor::accessPublicNh()
  {
    return this->publicNh_;
  }

  ros::NodeHandlePtr GeneralProcessor::accessProcessorNh()
  {
    return this->processorNh_;
  }

  std::string GeneralProcessor::getName()
  {
    return this->name_;
  }
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_GENERAL_PROCESSOR_H

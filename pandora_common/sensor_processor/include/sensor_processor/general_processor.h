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
    GeneralProcessor() {}
    virtual
    ~GeneralProcessor()
    {
      ROS_INFO("[%s] Destroyed", name_.c_str());
    }
    virtual void
    initialize(const std::string& ns, Handler* handler);

   protected:
    ros::NodeHandle& getPublicNodeHandle();
    ros::NodeHandle& getProcessorNodeHandle();
    std::string getName();

   protected:
    ros::NodeHandle public_nh_;
    ros::NodeHandle processor_nh_;
    std::string name_;
  };

  void
  GeneralProcessor::initialize(const std::string& ns, Handler* handler)
  {
    ros::NodeHandle private_nh = handler->getPrivateNh();
    std::string private_ns = private_nh.getNamespace();
    ROS_ASSERT(ns[0] == '~');
    std::string processor_ns = private_ns + "/" + ns.substr(1);
    this->processor_nh_ = ros::NodeHandle(processor_ns);
    this->public_nh_ = handler->getPublicNh();
    this->name_ = boost::to_upper_copy<std::string>(this->processor_nh_.getNamespace());
    ROS_INFO("[%s] Initialized", this->name_.c_str());
  }

  ros::NodeHandle&
  GeneralProcessor::
  getPublicNodeHandle()
  {
    return this->public_nh_;
  }

  ros::NodeHandle&
  GeneralProcessor::
  getProcessorNodeHandle()
  {
    return this->processor_nh_;
  }

  std::string
  GeneralProcessor::
  getName()
  {
    return this->name_;
  }

}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_GENERAL_PROCESSOR_H

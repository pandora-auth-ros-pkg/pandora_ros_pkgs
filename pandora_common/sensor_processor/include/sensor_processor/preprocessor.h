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
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef SENSOR_PROCESSOR_PREPROCESSOR_H
#define SENSOR_PROCESSOR_PREPROCESSOR_H

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include "sensor_processor/general_processor.h"
#include "sensor_processor/handler.h"

namespace sensor_processor
{
  template <class Input, class Output>
  class PreProcessor : public GeneralProcessor
  {
   private:
    typedef boost::shared_ptr<Input> InputPtr;
    typedef boost::shared_ptr<Input const> InputConstPtr;
    typedef boost::shared_ptr<Output> OutputPtr;

   public:
    PreProcessor(const std::string& ns, Handler* handler)
    {
      initialize(ns, handler);
    }

    PreProcessor() {}

    virtual bool
    preProcess(const InputConstPtr& input, const OutputPtr& output) = 0;

    virtual void
    initialize(const std::string& ns, Handler* handler)
    {
      GeneralProcessor::initialize(ns, handler);

      ros::NodeHandle private_nh = handler->getPrivateNh();

      XmlRpc::XmlRpcValue inputTopics;
      if (!private_nh.getParam("subscribed_topics", inputTopics))
      {
        ROS_FATAL("[%s] 'subscribed_topics:' param not found", this->getName().c_str());
        ROS_BREAK();
      }

      if (inputTopics.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        nSubscribers_.push_back(this->getPublicNodeHandle().subscribe(
              static_cast<std::string>(inputTopics), 1,
              static_cast<void(Handler::*)(const InputConstPtr&)>(&Handler::completeProcessCallback),
              handler));
      }
      else if (inputTopics.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int ii = 0; ii < inputTopics.size(); ii++) {
          ROS_ASSERT(inputTopics[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
          nSubscribers_.push_back(this->getPublicNodeHandle().subscribe(inputTopics[ii], 1,
            static_cast<void(Handler::*)(const InputConstPtr&)>(&Handler::completeProcessCallback),
            handler));
        }
      }
      else
      {
        ROS_FATAL("[%s] 'subscribed_topics' param can be either a string (single topic)"
          "or a list of strings (many topics)!", this->getName().c_str());
        ROS_ASSERT(false);
      }
    }

    bool
    process(boost::shared_ptr<boost::any> input,
        boost::shared_ptr<boost::any> output)
    {
      InputConstPtr in;
      OutputPtr out( new Output );
      try
      {
        in = boost::any_cast<InputConstPtr>(*input);
        *output = out;
      }
      catch (boost::bad_any_cast& e)
      {
        ROS_FATAL("[%s] Bad any_cast occured in preprocessor: %s",
            this->getName().c_str(), e.what());
        ROS_BREAK();
      }
      return preProcess(in, out);
    }

   private:
    std::vector<ros::Subscriber> nSubscribers_;
  };
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_PREPROCESSOR_H

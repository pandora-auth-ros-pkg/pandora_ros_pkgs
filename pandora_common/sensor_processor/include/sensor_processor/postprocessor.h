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

#ifndef SENSOR_PROCESSOR_POSTPROCESSOR_H
#define SENSOR_PROCESSOR_POSTPROCESSOR_H

#include <string>
#include <boost/shared_ptr.hpp>

#include "sensor_processor/general_processor.h"
#include "sensor_processor/handler.h"

namespace sensor_processor
{
  template <class Input, class Output>
  class PostProcessor : public GeneralProcessor
  {
   private:
    typedef boost::shared_ptr<Input> InputPtr;
    typedef boost::shared_ptr<Input const> InputConstPtr;
    typedef boost::shared_ptr<Output> OutputPtr;

   public:
    PostProcessor(const std::string& ns, Handler* handler)
    {
      initialize(ns, handler);
    }
    PostProcessor(void) {}

    virtual
    ~PostProcessor() {}

    virtual bool
    postProcess(const InputConstPtr& input, const OutputPtr& output) = 0;

    virtual void
    initialize(const std::string& ns, Handler* handler)
    {
      GeneralProcessor::initialize(ns, handler);

      std::string outputTopic;
      ros::NodeHandle privateNh("~");

      if (!privateNh.getParam("published_topics", outputTopic))
      {
        ROS_FATAL("[%s] 'published_topics:' param not found", this->getName().c_str());
        ROS_BREAK();
      }
      nPublisher_ = this->accessPublicNh()->template advertise<Output>(outputTopic, 1);
    }

    bool
    process(boost::shared_ptr<boost::any> input,
        boost::shared_ptr<boost::any> output)
    {
      InputConstPtr in;
      OutputPtr out( new Output );
      try
      {
        in = boost::any_cast<InputPtr>(*input);
        *output = out;
      }
      catch (boost::bad_any_cast& e)
      {
        ROS_FATAL("Bad any_cast occured in postprocessor %s: %s",
            this->getName().c_str(), e.what());
        ROS_BREAK();
      }

      bool success = postProcess(in, out);
      if (success)
      {
        nPublisher_.publish(*out);
      }
      return success;
    }

   private:
    ros::Publisher nPublisher_;
  };
}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_POSTPROCESSOR_H

/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors:
*   Tsirigotis Christos <tsirif@gmail.com>
*   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
*********************************************************************/

#ifndef SENSOR_PROCESSOR_HANDLER_H
#define SENSOR_PROCESSOR_HANDLER_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros/forwards.h>

#include "state_manager/state_client.h"
#include "sensor_processor/abstract_processor.h"

namespace sensor_processor
{
  class Handler : public StateClient
  {
   public:
    explicit Handler(const std::string& ns);
    virtual
    ~Handler();

    ros::NodeHandlePtr shareNodeHandle();

    template <class SubType>
    void completeProcessCallback(
          const boost::shared_ptr<SubType const>& subscribedTypePtr);

   protected:
    virtual void
    startTransition(int newState) {}
    virtual void
    completeTransition() {}

   private:
    void
    completeProcessFinish(bool success, const std::string& logInfo);

   protected:
    AbstractProcessorPtr preProcPtr_;
    AbstractProcessorPtr processorPtr_;
    AbstractProcessorPtr postProcPtr_;

    int currentState_;
    int previousState_;

    ros::NodeHandlePtr nhPtr_;
    ros::NodeHandle privateNh_;
    std::string name_;

   private:
    ros::Publisher operationReport_;
  };
}  // namespace sensor_processor

#include "sensor_processor/handler.hxx"

#endif  // SENSOR_PROCESSOR_HANDLER_H

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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <string>

#include <std_msgs/Int32.h>

#include "sensor_processor/test/dummy_handler.h"
#include "sensor_processor/test/dummy_preprocessor.h"
#include "sensor_processor/test/dummy_processor.h"
#include "sensor_processor/test/dummy_postprocessor.h"

namespace sensor_processor
{
  typedef std_msgs::Int32 Int32;
  class DummyHandler : public Handler
  {
  public:
    explicit DummyHandler(const std::string& ns);

  protected:
    virtual void
      startTransition(int newState);
    virtual void
      completeTransition();
  };

  DummyHandler::
  DummyHandler(const std::string& ns) : Handler(ns)
  {
  }

  void
  DummyHandler::
  startTransition(int newState)
  {
    currentState_ = newState;

    switch (currentState_)  // ................
    {
      case 2:
        preProcPtr_.reset( new DummyPreProcessor("~/preprocessor", this) );
        processorPtr_.reset( new DummyProcessor("~/processor", this) );
        postProcPtr_.reset( new DummyPostProcessor("~/postprocessor", this) );
        break;
      case state_manager_msgs::RobotModeMsg::MODE_TERMINATING:
        ros::shutdown();
        return;
    }
    previousState_ = currentState_;
    transitionComplete(currentState_);
  }

  void
  DummyHandler::
  completeTransition()
  {
  }
}  // namespace sensor_processor

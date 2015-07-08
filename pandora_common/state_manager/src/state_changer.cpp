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
 *   Software Architecture team
 * Maintainer:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <ros/ros.h>

#include "state_manager/state_client.h"

namespace state_manager
{
  class StateChanger : public StateClient
  {
   public:
    StateChanger() : StateClient(false) {}
    virtual ~StateChanger() {}

    virtual void
    startTransition(int newState)
    {
      ROS_INFO("[%s] changing to state %s", this->getName().c_str(),
          ROBOT_STATES(newState).c_str());
      transitionComplete(newState);
    }

    virtual void
    completeTransition() {}
  };

}  // namespace state_manager

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stateClient");
  state_manager::StateChanger state_changer;
	ros::Duration w(.1);
	w.sleep();

	ROS_INFO("Requested to change in state %s ", argv[1]);

	state_changer.transitionToState(atoi(argv[1]));
	ros::spin();
}

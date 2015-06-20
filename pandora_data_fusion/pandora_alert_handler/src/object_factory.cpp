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

#include "alert_handler/object_factory.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    ObjectFactory::ObjectFactory(const MapPtr& map, const std::string& mapType)
    {
      poseFinder_.reset( new PoseFinder(map, mapType) );
    }

    HolePtrVectorPtr ObjectFactory::makeHoles(
        const pandora_vision_msgs::HoleDirectionAlertVector& msg)
    {
      currentTransform_ = poseFinder_->lookupTransformFromWorld(msg.header);

      HolePtrVectorPtr holesVectorPtr( new HolePtrVector );
      for (int ii = 0; ii < msg.alerts.size(); ++ii)
      {
        try
        {
          HolePtr newHole( new Hole );
          setUpObject<Hole>(newHole, msg.alerts[ii], msg.header.stamp,
              currentTransform_);
          holesVectorPtr->push_back(newHole);
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER_OBJECT_FACTORY %d] %s", __LINE__, ex.what());
        }
      }

      return holesVectorPtr;
    }

    void ObjectFactory::dynamicReconfigForward(float occupiedCellThres,
        float highThres, float lowThres, float orientationCircle)
    {
      poseFinder_->updateParams(occupiedCellThres,
          highThres, lowThres, orientationCircle);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

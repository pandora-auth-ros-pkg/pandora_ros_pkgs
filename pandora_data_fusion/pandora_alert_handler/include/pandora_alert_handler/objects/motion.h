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

#ifndef PANDORA_ALERT_HANDLER_OBJECTS_MOTION_H
#define PANDORA_ALERT_HANDLER_OBJECTS_MOTION_H

#include <vector>

#include "pandora_common_msgs/GeneralAlertVector.h"
#include "pandora_common_msgs/GeneralAlertInfo.h"

#include "pandora_alert_handler/objects/object_interface/kalman_object.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  /**
    * @class Motion
    * @brief Concrete class representing a Motion Object. Inherits from Object
    */
  class Motion : public KalmanObject<Motion>
  {
   public:
    //!< Type Definitions
    typedef pandora_common_msgs::GeneralAlertInfo Alert;
    typedef pandora_common_msgs::GeneralAlertVector AlertVector;

   public:
    static void setUpObject(const Ptr& ptr, const Alert& msg)
    {
      return;
    }

    /**
      * @brief Constructor
      */
    Motion();

    virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
  };

  typedef Motion::Ptr MotionPtr;
  typedef Motion::ConstPtr MotionConstPtr;
  typedef Motion::PtrVector MotionPtrVector;
  typedef Motion::PtrVectorPtr MotionPtrVectorPtr;
  typedef Motion::List MotionList;
  typedef Motion::ListPtr MotionListPtr;
  typedef Motion::ListConstPtr MotionListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_OBJECTS_MOTION_H

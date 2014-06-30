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

#ifndef ALERT_HANDLER_TF_LISTENER_H
#define ALERT_HANDLER_TF_LISTENER_H

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    class TfListener
    {
      public:
        //!< Type Definitions
        typedef boost::shared_ptr<TfListener> Ptr;
        typedef boost::shared_ptr<TfListener const> ConstPtr;

      public:
        TfListener() {}

        virtual bool waitForTransform(const std::string& target_frame,
            const std::string& source_frame, const ros::Time& time,
            const ros::Duration& timeout,
            const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
            std::string* error_msg = NULL) const
        {
          return true;
        }

        virtual void lookupTransform(const std::string& target_frame,
            const std::string& source_frame, const ros::Time& time,
            tf::StampedTransform& transform) const
        {
          transform.setOrigin(tf::Vector3(5, 5, 0.3));
          transform.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
        }
    };

    typedef TfListener::Ptr TfListenerPtr;
    typedef TfListener::ConstPtr TfListenerConstPtr;

    class RosTfListener: public TfListener
    {
      public:
        RosTfListener();

        bool waitForTransform(const std::string& target_frame,
            const std::string& source_frame, const ros::Time& time,
            const ros::Duration& timeout,
            const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
            std::string* error_msg = NULL) const;
        void lookupTransform(const std::string& target_frame,
            const std::string& source_frame, const ros::Time& time,
            tf::StampedTransform& transform) const;

      private:
        tf::TransformListener listener;
    };

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_TF_LISTENER_H

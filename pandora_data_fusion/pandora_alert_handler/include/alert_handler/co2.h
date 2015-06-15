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

#ifndef ALERT_HANDLER_CO2_H
#define ALERT_HANDLER_CO2_H

#include <vector>

#include "pandora_common_msgs/GeneralAlertVector.h"
#include "pandora_common_msgs/GeneralAlertInfo.h"

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Co2
     * @brief Concrete class representing a Co2 Object. Inherits from Object
     */
    class Co2 : public KalmanObject<Co2>
    {
      public:
        //!< Type Definitions
        typedef boost::shared_ptr<Co2> Ptr;
        typedef boost::shared_ptr<Co2 const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef ObjectList<Co2> List;
        typedef boost::shared_ptr<List> ListPtr;
        typedef boost::shared_ptr< const ObjectList<Co2> > ListConstPtr;

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
        Co2();

        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
    };

    typedef Co2::Ptr Co2Ptr;
    typedef Co2::ConstPtr Co2ConstPtr;
    typedef Co2::PtrVector Co2PtrVector;
    typedef Co2::PtrVectorPtr Co2PtrVectorPtr;
    typedef Co2::List Co2List;
    typedef Co2::ListPtr Co2ListPtr;
    typedef Co2::ListConstPtr Co2ListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_CO2_H

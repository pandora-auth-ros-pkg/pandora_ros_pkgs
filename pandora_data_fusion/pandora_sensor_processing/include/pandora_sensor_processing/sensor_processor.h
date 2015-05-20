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

#ifndef SENSOR_PROCESSING_SENSOR_PROCESSOR_H
#define SENSOR_PROCESSING_SENSOR_PROCESSOR_H

#ifndef BOOST_NO_DEFAULTED_FUNCTIONS
#define BOOST_NO_DEFAULTED_FUNCTIONS
#endif

#include <string>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include "state_manager/state_client.h"
#include "pandora_common_msgs/GeneralAlert.h"
#include "pandora_common_msgs/GeneralAlertInfo.h"
#include "pandora_common_msgs/GeneralAlertVector.h"
#include "pandora_sensor_processing/SensorProcessingConfig.h"

namespace pandora_sensor_processing
{

  /**
   * @brief Abstraction layer that is responsible for communication over ROS
   * with other nodes and processor organization through StateClient.
   */
  template <class DerivedProcessor>
    class SensorProcessor
    : public StateClient, private boost::noncopyable
    {
      public:
        /**
         * @brief Constructor
         * @param ns [std::string const&] Has the namespace of the node.
         * @param sensorType [std::string const&] Name of sensor
         */
        SensorProcessor(const std::string& ns, const std::string& sensorType);

        void startTransition(int newState);

        void completeTransition();

        /**
         * @brief getter for general alert message alert_
         * @return pandora_common_msgs::GeneralAlert alert_
         */
        pandora_common_msgs::GeneralAlert getAlert() const
        {
          return alert_;
        }

      protected:
        /**
         * @brief Delegates to alertPublisher_.
         * @return void
         */
        virtual void publishAlert();

      private:
        /**
         * @brief Toggles subscriber on, if toWork is true, or off else.
         * @param toWork [bool] is sensor going to work?
         * @return void
         */
        void toggleSubscriber(bool toWork);

      protected:
        //!< Alert message that includes info to be filled by Derived Processor.
        pandora_common_msgs::GeneralAlert alert_;

        //!< Name of this Sensor Processor.
        std::string name_;

      private:
        ros::NodeHandle nh_;

        //!< Contains this processor's sensor type.
        std::string sensorType_;

        //!< Publisher for the final product of sensor processing.
        ros::Publisher alertPublisher_;
        //!< Topic to which alerts are being published.
        std::string publisherTopic_;

        //!< Subscriber for raw input data from hardware interface.
        ros::Subscriber sensorSubscriber_;
        //!< Topic to which sensor processor is listening.
        std::string subscriberTopic_;
        //!< is sensor currently working?
        bool working_;

        dynamic_reconfigure::Server< SensorProcessingConfig >
          dynReconfServer_;

        /*  Parameters  */
        //!< Is sensor open in exploration state?
        bool EXPLORATION_STATE;
        //!< Is sensor open in identification state?
        bool IDENTIFICATION_STATE;
        //!< Is sensor open in sensor hold state?
        bool SENSOR_HOLD_STATE;
    };

}  // namespace pandora_sensor_processing

#include "pandora_sensor_processing/sensor_processor.hxx"

#endif  // SENSOR_PROCESSING_SENSOR_PROCESSOR_H

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

#ifndef SENSOR_COVERAGE_SENSOR_COVERAGE_H
#define SENSOR_COVERAGE_SENSOR_COVERAGE_H

#ifndef BOOST_NO_DEFAULTED_FUNCTIONS
#define BOOST_NO_DEFAULTED_FUNCTIONS
#endif

#include <string>
#include <vector>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "octomap_msgs/Octomap.h"

#include "state_manager/state_client.h"

#include "sensor_coverage/sensor.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    //!< Type Definitions
    typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
    typedef boost::shared_ptr<octomap_msgs::Octomap> OctomapPtr;

    class SensorCoverage
      : public StateClient, private boost::noncopyable
    {
      public:
        /**
         * @brief Constructor
         * @param ns [std::string const&] Has the namespace of the node.
         */
        explicit SensorCoverage(const std::string& ns);

        /**
         * @override
         * @brief Callback for every state change that occurs in state server.
         * It is used to make changes according to the state we are entering.
         * @param newState [int] number that indicates the state
         * @return void
         */
        void startTransition(int newState);

        /**
         * @override
         * @brief Callback that activates when state transition has occured for
         * all state clients.
         * @return void
         */
        void completeTransition();

      private:
        /**
         * @brief mapSubscriber_'s callback to copy map from SLAM
         * @param map [octomap_msgs::Octomap const&] fetched map
         * @return void
         */
        void mapUpdate(const octomap_msgs::Octomap& map);

      private:
        //!< This node's NodeHandle.
        NodeHandlePtr nh_;

        //!< subscriber that fetches map.
        ros::Subscriber mapSubscriber_;

        //!< Robot's current mode of operation
        int currentState_;
        //!< 3d map recieved from SLAM.
        OctomapPtr globalMap_;
        //!< vector containing all sensors registered to track their views and
        //!< make their coverage patches.
        std::vector<SensorPtr> registeredSensors_;
    };

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

#endif  // SENSOR_COVERAGE_SENSOR_COVERAGE_H
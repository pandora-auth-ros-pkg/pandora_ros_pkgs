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

#ifndef SENSOR_COVERAGE_SENSOR_H
#define SENSOR_COVERAGE_SENSOR_H

#include <string>
#include <boost/shared_ptr.hpp>

#include "octomap_msgs/Octomap.h"

#include "state_manager_communications/robotModeMsg.h"

#include "alert_handler/tf_finder.h"
#include "alert_handler/tf_listener.h"
#include "alert_handler/exceptions.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    //!< Type Definitions
    using ::pandora_data_fusion::pandora_alert_handler::TfFinder;
    using ::pandora_data_fusion::pandora_alert_handler::TfListener;
    using ::pandora_data_fusion::pandora_alert_handler::TfListenerPtr;
    using ::pandora_data_fusion::pandora_alert_handler::TfException;
    typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
    typedef boost::shared_ptr<octomap_msgs::Octomap> OctomapPtr;

    /**
     * @brief class that correspond to a tracked sensor.
     * Contains methods that draw coverage into current map.
     */
    class Sensor
    {
      public:
        /**
         * @brief constructor for sensor class
         * @param nh [NodeHandlePtr const&] pointer to node's nodehandle
         * @param globalMap [OctomapPtr const&] pointer to global octomap
         * @param frameName [std::string const&] frame whose view is to be tracked
         * @param mapOrigin [std::string const&] map's origin (SLAM or TEST)
         */
        Sensor(const NodeHandlePtr& nh, const OctomapPtr& globalMap,
            const std::string& frameName, const std::string& mapOrigin);

        /**
         * @brief notifies state change
         * @param newState [int] the state to which sensor is transitioning
         * @return void
         */
        void notifyStateChange(int newState);

      protected:
        /**
         * @brief callback for timer that updates sensor's coverage patch
         * @param event [ros::TimerEvent const&]
         * @return void
         */
        void coverageUpdate(const ros::TimerEvent& event);

        /**
         * @brief function that draws upon coverage patch, triggered when updating it
         * @return void
         */
        void patchDrawer();

      private:
        /**
         * @brief Getter for sensor's parameters
         * @return void
         */
        void getParameters();

      protected:
        //!< Node's shared NodeHandle
        NodeHandlePtr nh_;

        //!< Publisher for this sensor's coverage patch
        ros::Publisher coveragePublisher_;
        //!< Timer that triggers updating of the coverage patch
        ros::Timer coverageUpdater_;

        //!< is sensor open and working?
        bool sensorWorking_;

        //!< Global 3d map as it sent from SLAM
        OctomapPtr globalMap_;

        //!< Sensor's tf frame which is being tracked
        std::string frameName_;
        //!< Abstract transformation listener.
        TfListenerPtr listener_;
        //!< Current sensor's transformation stamped
        tf::StampedTransform tfTransform_;
        //!< Sensor's coverage patch (this object's output)
        octomap_msgs::Octomap coveragePatch_;

        /*  Parameters  */
        //!< sensor's range
        double SENSOR_RANGE;
        //!< sensor's horizontal field of view
        double SENSOR_HFOV;
        //!< sensor's vertical field of view
        double SENSOR_VFOV;

        /*  Sensor's state: True if open, False if closed  */
        //!< sensor's state in EXPLORATION_MODE
        bool EXPLORATION_STATE;
        //!< sensor's state in IDENTIFICATION_MODE
        bool IDENTIFICATION_STATE;
        //!< sensor's state in HOLD_MODE
        bool HOLD_STATE;

      private:
        friend class SensorTest;
    };

    typedef boost::shared_ptr<Sensor> SensorPtr;

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

#endif  // SENSOR_COVERAGE_SENSOR_H

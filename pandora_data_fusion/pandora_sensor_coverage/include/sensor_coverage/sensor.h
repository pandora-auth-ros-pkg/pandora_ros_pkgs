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

#include "state_manager_communications/robotModeMsg.h"

#include "alert_handler/tf_finder.h"
#include "alert_handler/tf_listener.h"
#include "alert_handler/exceptions.h"

#include "sensor_coverage/space_checker.h"
#include "sensor_coverage/surface_checker.h"

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
         * @param frameName [std::string const&] frame whose view is to be tracked
         * @param mapOrigin [std::string const&] map's origin (SLAM or TEST)
         */
        Sensor(const NodeHandlePtr& nh,
            const std::string& frameName, const std::string& mapOrigin);

        /**
         * @brief notifies state change
         * @param newState [int] the state to which sensor is transitioning
         * @return void
         */
        void notifyStateChange(int newState);

        /**
         * @brief Setter for static variable map2d_
         * @param map2d [nav_msgs::OccupancyGridPtr const&] map
         * @return void
         */
        static void setMap2d(const nav_msgs::OccupancyGridPtr& map2d)
        {
          map2d_ = map2d;
          CoverageChecker::setMap2d(map2d);
        }

        /**
         * @brief Setter for static variable map3d_
         * @param map3d [boost::shared_ptr<octomap::OcTree> const&] map
         * @note Will reset to null, deleting reference, if a null ptr is passed.
         * @return void
         */
        static void setMap3d(const boost::shared_ptr<octomap::OcTree>& map3d)
        {
          map3d_ = map3d;
          CoverageChecker::setMap3d(map3d);
        }

        /**
         * @brief Setter for static global static frame.
         * @param globalFrame [std::string const&] static frame of map
         * @return void
         */
        static void setGlobalFrame(const std::string& globalFrame)
        {
          GLOBAL_FRAME = globalFrame;
        }

        /**
         * @brief Setter for robot's base frame.
         * @param robotBaseFrame [std::string const&] base footprint
         * @return void
         */
        static void setRobotBaseFrame(const std::string& robotBaseFrame)
        {
          ROBOT_BASE_FRAME = robotBaseFrame;
        }

        /**
         * @brief delegate to coverage checker
         * @param occupiedCellThres [double] threshold
         * @return void
         */
        static void setOccupiedCellThres(double occupiedCellThres)
        {
          CoverageChecker::setOccupiedCellThres(occupiedCellThres);
        }

      protected:
        /**
         * @brief callback for timer that updates sensor's coverage patch
         * @param event [ros::TimerEvent const&]
         * @return void
         */
        void coverageUpdate(const ros::TimerEvent& event);

      private:
        /**
         * @brief Getter for sensor's parameters
         * @return void
         */
        void getParameters();

      protected:
        //!< Node's shared NodeHandle.
        NodeHandlePtr nh_;

        //!< Timer that triggers updating of the surface coverage patch
        //!< and space coverage map.
        ros::Timer coverageUpdater_;

        //!< is sensor open and working?
        bool sensorWorking_;
        //!< Sensor's tf frame which is being tracked.
        std::string frameName_;
        //!< Is surface coverage needed?
        bool surfaceCoverage_;

        //!< Abstract transformation listener.
        TfListenerPtr listener_;

        //!< Space coverage finder
        boost::scoped_ptr<SpaceChecker> spaceChecker_;
        //!< Surface coverage finder
        boost::scoped_ptr<SurfaceChecker> surfaceChecker_;

        //!< Global 3d and 2d maps as they are sent by SLAM
        static boost::shared_ptr<octomap::OcTree> map3d_;
        static nav_msgs::OccupancyGridPtr map2d_;

        /*  Params  */
        static std::string GLOBAL_FRAME;
        static std::string ROBOT_BASE_FRAME;

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

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
 *   Lykartsis Ioannis <lykartsis.giannis@gmail.com>
 *********************************************************************/

#ifndef SENSOR_COVERAGE_PLANNER_COVERAGE_H
#define SENSOR_COVERAGE_PLANNER_COVERAGE_H

#ifndef BOOST_NO_DEFAULTED_FUNCTIONS
#define BOOST_NO_DEFAULTED_FUNCTIONS
#endif

#include <string>
#include <vector>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include "octomap/octomap.h"
#include "octomap_msgs/Octomap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "state_manager/state_client.h"

namespace pandora_exploration
{
  namespace pandora_sensor_planner
  {


    //!< Type Definitions
    typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;

    class SensorPlanner
      : public StateClient, private boost::noncopyable
    {
      public:
        /**
         * @brief Constructor
         * @param ns [std::string const&] Has the namespace of the node.
         */
        explicit SensorPlanner(const std::string& ns);

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

        /**
         * @brief Setter for static variable coveredSurface_
         * @param map3d [boost::shared_ptr<octomap::OcTree> const&] map
         * @note Will reset to null, deleting reference, if a null ptr is passed.
         * @return void
         */
        void setMap3d(const boost::shared_ptr<octomap::OcTree>& map3d)
        {
          coveredSurface_ = map3d;
        }

        

       
      private:
        /**
         * @brief map3dSubscriber_'s callback to copy map from
         * surface_checker
         * @param msg [octomap_msgs::Octomap const&] fetched map
         * @return void
         */
        void octomapUpdate(const octomap_msgs::Octomap& msg);

        /**
         * @brief map2dsubscriber_'s callback to copy map from SLAM
         * @param msg [nav_msgs::OccupancyGridConstPtr const&] fetched map
         * @return void
         */
        void occupancyGridUpdate(const nav_msgs::OccupancyGridConstPtr& msg);

      private:
         //!< This node's NodeHandle.
         NodeHandlePtr nh_;

         //!< subscriber that fetches octomap.
         ros::Subscriber octomapSubscriber_;
         //!< subscriber that fetches occupancy grid.
         ros::Subscriber occupancyGridSubscriber_;

         //!< Is sensor planning active at this state?
         bool sensorPlanning_;

         //!< Robot's current mode of operation.
         int currentState_;
         //!< Octomap received from sensor coverage
         octomap_msgs::Octomap coveredSurface_;
         //!< Occupancy Grid reveived from sensor coverage
         nav_msgs::OccupancyGridPtr coveredSpace_;
       };
}   // namespace pandora_sensor_planner
}   // namespace pandora_exploration

#endif  // SENSOR_COVERAGE_SENSOR_PLANNER_H






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

#ifndef SENSOR_COVERAGE_COVERAGE_CHECKER_H
#define SENSOR_COVERAGE_COVERAGE_CHECKER_H

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "tf/transform_datatypes.h"
#include "octomap_ros/conversions.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/Octomap.h"
#include "octomap/octomap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    //!< Type Definitions
    using ::pandora_data_fusion::pandora_alert_handler::Utils;
    typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;

    /**
     * @brief Base class for a coverage checker
     */
    class CoverageChecker
    {
      public:
        /**
         * @brief Constructor
         */
        CoverageChecker(const NodeHandlePtr& nh, const std::string& frameName);

        /**
         * @brief function that finds coverage, triggered when updating it
         * @param transform [tf::StampedTransform const&] tf used in coverage finding
         * @return void
         */
        virtual void findCoverage(const tf::StampedTransform& transform);

        /**
         * @brief publishes coverage map or patch
         * @return void
         */
        virtual void publishCoverage() {}

        /**
         * @brief Setter for static variable map2D_
         * @param map2D [nav_msgs::OccupancyGridPtr const&] map
         * @return void
         */
        static void setMap2d(const nav_msgs::OccupancyGridPtr& map2d)
        {
          map2d_ = map2d;
        }

        /**
         * @brief Setter of static variable 3dMap_
         * @param map3d [boost::shared_ptr<octomap::OcTree> const&] map
         * @note Will reset to null, deleting reference, if a null ptr is passed.
         * @return void
         */
        static void setMap3d(const boost::shared_ptr<octomap::OcTree>& map3d)
        {
          map3d_ = map3d;
        }

        /**
         * @brief Setter for static variable OCCUPIED_CELL_THRES
         * @param occupiedCellThres [double] threshold
         * @return void
         */
        static void setOccupiedCellThres(double occupiedCellThres)
        {
          OCCUPIED_CELL_THRES = occupiedCellThres;
        }

      protected:
        /**
         * @brief Getter for sensor's parameters
         * @return void
         */
        virtual void getParameters();

      protected:
        //!< Node's NodeHandle
        NodeHandlePtr nh_;
        //!< Name of tracked sensor's frame
        std::string frameName_;
        //!< Publisher for this sensor's surface coverage.
        ros::Publisher coveragePublisher_;
        //!< Useful for coverage finding.
        double roll_;
        double pitch_;
        double yaw_;
        octomap::point3d position_;

        //!< Global 3d and 2d maps as they are sent by SLAM
        static boost::shared_ptr<octomap::OcTree> map3d_;
        static nav_msgs::OccupancyGridPtr map2d_;

        /*  Parameters  */
        //!< sensor's range
        double SENSOR_RANGE;
        //!< sensor's horizontal field of view
        double SENSOR_HFOV;
        //!< sensor's vertical field of view
        double SENSOR_VFOV;
        //!< 2d map's occupancy threshold
        static double OCCUPIED_CELL_THRES;
    };

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

#endif  // SENSOR_COVERAGE_COVERAGE_CHECKER_H



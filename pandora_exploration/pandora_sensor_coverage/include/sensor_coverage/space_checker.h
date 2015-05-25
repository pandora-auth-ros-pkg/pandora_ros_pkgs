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

#ifndef SENSOR_COVERAGE_SPACE_CHECKER_H
#define SENSOR_COVERAGE_SPACE_CHECKER_H

#include <string>
#include <boost/shared_ptr.hpp>

#include "sensor_coverage/coverage_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    /**
     * @brief class that correspond to a tracked sensor.
     * Contains methods that draw coverage into current map.
     */
    template <class TreeType>
      class SpaceChecker : public CoverageChecker
      {
        public:
          /**
           * @brief constructor for sensor class
           * @param nh [NodeHandlePtr const&] pointer to node's nodehandle
           * @param frameName [std::string const&] frame whose view is to be tracked
           */
          SpaceChecker(const NodeHandlePtr& nh, const std::string& frameName);

          /**
           * @override
           * @brief function that finds coverage, triggered when updating it
           * @param sensorTransform [tf::StampedTransform const&] tf that will be used
           * in coverage finding
           * @param baseTransform [tf::StampedTransform const&] base footprint's tf
           * @return void
           */
          virtual void findCoverage(const tf::StampedTransform& sensorTransform,
              const tf::StampedTransform& baseTransform);

          /**
           * @override
           * @brief publishes coverage map or patch
           * @param frame [std::string const&] frame id of the coverage.
           * @return void
           */
          virtual void publishCoverage(const std::string& frame);

          /**
           * @override
           * @brief resets coverage map.
           * @return void
           */
          virtual void resetCoverage();

          /**
           * @brief Setter for variable coverageMap3d_
           * @param map [boost::shared_ptr<TreeType> const&]
           * map which will be projected down to find area and space coverage.
           * @return void
           */
          void setCoverageMap3d(const boost::shared_ptr<TreeType>& map)
          {
            coverageMap3d_ = map;
          }

        private:
          /**
           * @brief finds cell's space coverage as a percentage of the covered
           * space above it
           * @param cell [octomath::Vector3 const&] cell in question
           * @param minHeight [float] minimun height of interest (base footprint)
           * @return float percentage of space covered by sensor.
           */
          float cellCoverage(const octomath::Vector3& cell, float minHeight);

          /**
           * @brief aligns coverage map with current global world map. Rotates,
           * translates and scales coverage map appropriately.
           * @return void
           */
          void alignCoverageWithMap();

          /**
           * @brief Wrapper for pandora_vision::Morphology::dilation()
           * @return void
           */
          void coverageDilation(int steps, int coords);

        protected:
          //!< If space coverage is considered as a binary value or as a percentage.
          bool binary_;
          //!< Do we use for 3d map, the produced map by surface checker?
          bool surfaceCoverage_;
          //!< Sensor's space coverage map.
          nav_msgs::OccupancyGridPtr coveredSpace_;

          //!< 3d coverage map produced when surfacing checking.
          boost::shared_ptr<TreeType> coverageMap3d_;

          //!< Total area covered with this sensor.
          float totalAreaCovered_;
          //!< publishes total area covered with this sensor.
          ros::Publisher areaCoveragePublisher_;

          /*  Parameters  */

        private:
          friend class SpaceCheckerTest;
      };

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

#include "sensor_coverage/space_checker.hxx"

#endif  // SENSOR_COVERAGE_SPACE_CHECKER_H


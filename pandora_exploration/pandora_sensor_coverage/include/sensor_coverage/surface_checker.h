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

#ifndef SENSOR_COVERAGE_SURFACE_CHECKER_H
#define SENSOR_COVERAGE_SURFACE_CHECKER_H

#include <string>
#include <utility>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "octomap/ColorOcTree.h"

#include "sensor_coverage/coverage_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    /**
     * @brief class that contains methods to find surface coverage.
     * Keeps surface coverage patch.
     */
    class SurfaceChecker : public CoverageChecker
    {
      public:
        /**
         * @brief Constructor for surface checker class
         * @param nh [NodeHandlePtr const&] pointer to node's nodehandle
         * @param frameName [std::string const&] frame whose view is to be tracked
         */
        SurfaceChecker(const NodeHandlePtr& nh, const std::string& frameName);

        /**
         * @override
         * @brief function that finds coverage, triggered when updating it
         * @param sensorTransform [tf::StampedTransform const&] tf that will be used
         * in coverage finding
         * @param baseTransform [tf::StampedTransform const&] base footprint's tf
         * @return void
         */
        virtual void findCoverage(
            const tf::StampedTransform& sensorTransform,
            const tf::StampedTransform& baseTransform);

        /**
         * @override
         * @brief publishes coverage map or patch
         * @param frame [std::string const&] frame id of the tree.
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
         * @brief Setter for static variable ORIENTATION_CIRCLE
         * @param orientationCircle [double] radius in which a surface
         * is considered planar.
         * @return void
         */
        static void setOrientationCircle(double orientationCircle)
        {
          ORIENTATION_CIRCLE = orientationCircle;
        }

        /**
         * @brief Getter checker's coverage map
         * @return boost::shared_ptr<octomap::ColorOcTree>
         */
        boost::shared_ptr<octomap::ColorOcTree> getCoverageMap3d() const
        {
          return coveredSurface_;
        }

      protected:
        /**
         * @brief Metric that finds the coverage of a covered point on a surface.
         * @note By default it is estimated as the flux of the ray that traced it.
         * @param normalOnWall [octomap::point3d const&] normal vector on a surface.
         * @param direction [octomap::point3d const&] direction at which we are
         * facing the point
         * @return unsigned char dot product of two vectors scaled on 255.
         */
        virtual unsigned char metric(const octomap::point3d& normalOnWall,
            const octomap::point3d& direction);

      private:
        /**
         * @brief finds pointOnWall's respective coverage.
         * @param pointOnWall [octomap::point3d const&] a covered point on a surface.
         * @param direction [octomap::point3d const&] direction at which we are
         * facing the point.
         * @return unsigned char estimation of point's coverage.
         */
        unsigned char findPointCoverage(const octomap::point3d& pointOnWall,
            const octomap::point3d& direction);

        /**
         * @brief finds a normal (unit) vector on wall of the 3d map at the given point.
         * @param point [octomap::point3d const&] point on wall
         * @param normal [octomap::point3d*] normal vector to be found
         * @return bool whether operation was successful or not.
         */
        bool findNormalVectorOnWall(const octomap::point3d& point,
            octomap::point3d* normal);

        /**
         * @brief finds from a collection of points the two that are farther apart.
         * @param points [std::vector<octomap::point3d> const&] collection of points
         * @return std::pair<octomap::point3d, octomap::point3d> these two points.
         */
        std::pair<octomap::point3d, octomap::point3d> findDiameterEndPointsOnWall(
            const std::vector<octomap::point3d>& points);

      protected:
        //!< If surface coverage is considered as a binary value or as a percentage.
        bool binary_;
        //!< Factor by which surface coverage's resolution is lowered compared to
        //!< map3d_'s resolution.
        double blurFactor_;
        //!< Sensor's surface coverage patch.
        boost::shared_ptr<octomap::ColorOcTree> coveredSurface_;

        /*  Parameters  */
        //!< Radius in which a surface is considered to be planar.
        static double ORIENTATION_CIRCLE;

      private:
        friend class SurfaceCheckerTest;
    };

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

#endif  // SENSOR_COVERAGE_SURFACE_CHECKER_H


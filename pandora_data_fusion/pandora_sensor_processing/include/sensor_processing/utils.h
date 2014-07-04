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

#ifndef SENSOR_PROCESSING_UTILS_H
#define SENSOR_PROCESSING_UTILS_H

#ifndef BOOST_NO_DEFAULTED_FUNCTIONS
#define BOOST_NO_DEFAULTED_FUNCTIONS
#endif

#include <utility>
#include <vector>
#include <cmath>
#include <stdexcept>

#include <boost/math/constants/constants.hpp>
#include <boost/utility.hpp>

#include <Eigen/Dense>

//!< Macro for pi.
#define PI boost::math::constants::pi<float>()

namespace pandora_sensor_processing
{

  /**
   * @brief class containing mathematical tools and useful functions.
   */
  class Utils : private boost::noncopyable
  {
    public:
      /**
       * @param vec [Eigen::Vector4f const&] vector to find its mahal. 
       * distance from a group
       * @param mean [Eigen::Vector4fi const&] group's mean
       * @param covariance [Eigen::Matrix4f const&] group's covariance
       * @return float vec's mahalanobis distance to group
       */
      static float
        getMahalanobisDistance(const Eigen::Vector4f& vec,
            const Eigen::Vector4f& mean,
            const Eigen::Matrix4f& covariance);

      /**
       * @brief normal distribution function
       * @param x [float] the random variable
       * @param mean [float] X's mean value (pdf's peak)
       * @param stdDev [float] X's standard deviation
       * @return float pdf's corresponding value
       */
      static float normalPdf(float x, float mean, float stdDev);

      /**
       * @brief weibull distribution function
       * @param x [float] the random variable
       * @param k [float] pdf's shape value
       * @param l [float] X's scale
       * @return float pdf's corresponding value
       */
      static double weibullPdf(double x, double k, double l);
  };

}  // namespace pandora_sensor_processing

#endif  // SENSOR_PROCESSING_UTILS_H


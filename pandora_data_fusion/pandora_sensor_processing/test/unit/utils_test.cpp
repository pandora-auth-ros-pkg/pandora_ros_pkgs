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

#include "gtest/gtest.h"

#include "sensor_processing/utils.h"

namespace pandora_sensor_processing
{

    TEST(getMahalanobisDistance, checkResultsOfLegitValues)
    {
      Eigen::Vector4f vec, mean;
      Eigen::Matrix4f cov;

      vec << 1, 0, 0, 0;
      mean << 0, 0, 0, 0;
      cov = Eigen::MatrixXf::Identity(4, 4);
      EXPECT_NEAR(1, Utils::getMahalanobisDistance(vec, mean, cov), 0.0001);

      vec << 3, 0, 0, 0;
      mean << 1, 0, 0, 0;
      cov = Eigen::MatrixXf::Identity(4, 4);
      EXPECT_NEAR(2, Utils::getMahalanobisDistance(vec, mean, cov), 0.0001);

      vec << 1, 0, 0, 0;
      mean << 0, 0, 0, 0;
      cov = 2 * Eigen::MatrixXf::Identity(4, 4);
      EXPECT_NEAR(sqrt(2)/2, Utils::getMahalanobisDistance(vec, mean, cov), 0.0001);
    }

    TEST(normalPdf, checkResultsOfLegitValues)
    {
      EXPECT_NEAR(0.39894, Utils::normalPdf(0, 0, 1), 0.0001);
      EXPECT_NEAR(0.053991, Utils::normalPdf(2, 0, 1), 0.0001);
      EXPECT_NEAR(0.19947, Utils::normalPdf(0, 0, 2), 0.0001);
      EXPECT_NEAR(0.21297, Utils::normalPdf(35, 36, 1.5), 0.0001);
      EXPECT_NEAR(0, Utils::normalPdf(45, 36, 2), 0.0001);
      EXPECT_NEAR(0.0022159, Utils::normalPdf(30, 36, 2), 0.0001);
      EXPECT_NEAR(0, Utils::normalPdf(25, 36, 2), 0.0001);
    }

    TEST(weibullPdf, checkResultsOfLegitValues)
    {
      EXPECT_NEAR(0.36788, Utils::weibullPdf(1, 1, 1), 0.0001);
      EXPECT_NEAR(0.73576, Utils::weibullPdf(1, 2, 1), 0.0001);
      EXPECT_NEAR(0.18394, Utils::weibullPdf(1, 0.5, 1), 0.0001);
      EXPECT_NEAR(0.061313, Utils::weibullPdf(3, 0.5, 3), 0.0001);
      EXPECT_NEAR(0.23352, Utils::weibullPdf(3, 5, 4.3), 0.0001);
      EXPECT_NEAR(0.50998, Utils::weibullPdf(0.8, 1.8, 0.5), 0.0001);
    }

}  // namespace pandora_sensor_processing


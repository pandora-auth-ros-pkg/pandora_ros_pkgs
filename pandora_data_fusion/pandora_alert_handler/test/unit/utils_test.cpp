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
 *   Chamzas Konstantinos <chamzask@gmail.com>
 *********************************************************************/

#include "gtest/gtest.h"

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    TEST(probabilityFromStdDev, checkResultsOfLegitValues)
    {
      EXPECT_NEAR(0.019801, Utils::probabilityFromStdDev(0.1, 0.5), 0.0001);
      EXPECT_NEAR(0.39347, Utils::probabilityFromStdDev(0.5, 0.5), 0.0001);
      EXPECT_NEAR(0.86466, Utils::probabilityFromStdDev(1, 0.5), 0.0001);
      EXPECT_NEAR(0.98889, Utils::probabilityFromStdDev(1.5, 0.5), 0.0001);
      EXPECT_NEAR(0.99966, Utils::probabilityFromStdDev(2, 0.5), 0.0001);

      EXPECT_NEAR(1, Utils::probabilityFromStdDev(1, 0.1), 0.0001);
      EXPECT_NEAR(0.99613, Utils::probabilityFromStdDev(1, 0.3), 0.0001);
      EXPECT_NEAR(0.86466, Utils::probabilityFromStdDev(1, 0.5), 0.0001);
      EXPECT_NEAR(0.58889, Utils::probabilityFromStdDev(1, 0.75), 0.0001);
      EXPECT_NEAR(0.39347, Utils::probabilityFromStdDev(1, 1), 0.0001);
      EXPECT_NEAR(0.19926, Utils::probabilityFromStdDev(1, 1.5), 0.0001);
      EXPECT_NEAR(0.11750, Utils::probabilityFromStdDev(1, 2), 0.0001);
    }

    TEST(probabilityFromStdDev, wrongInputValues)
    {
      EXPECT_THROW(Utils::probabilityFromStdDev(-1, 0.5), std::range_error);
      EXPECT_THROW(Utils::probabilityFromStdDev(0, 0.5), std::range_error);
      EXPECT_THROW(Utils::probabilityFromStdDev(1, -0.5), std::range_error);
    }

    TEST(probabilityFromStdDev, returnsValueFor0)
    {
      EXPECT_THROW(Utils::probabilityFromStdDev(0, 0), std::range_error);
      EXPECT_NEAR(1, Utils::probabilityFromStdDev(0.5, 0), 0.0001);
      EXPECT_NEAR(1, Utils::probabilityFromStdDev(1, 0), 0.0001);
      EXPECT_NEAR(1, Utils::probabilityFromStdDev(30, 0), 0.0001);
    }

    TEST(stdDevFromProbability, checkResultsOfLegitValues)
    {
      EXPECT_NEAR(0.5, Utils::stdDevFromProbability(0.1, 0.019801), 0.0001);
      EXPECT_NEAR(0.5, Utils::stdDevFromProbability(0.5, 0.39347), 0.0001);
      EXPECT_NEAR(0.5, Utils::stdDevFromProbability(1, 0.86466), 0.0001);
      EXPECT_NEAR(0.5, Utils::stdDevFromProbability(1.5, 0.98889), 0.0001);
      EXPECT_NEAR(0.5, Utils::stdDevFromProbability(2, 0.99966), 0.001);

      EXPECT_NEAR(0, Utils::stdDevFromProbability(1, 1), 0.0001);
      EXPECT_NEAR(0, Utils::stdDevFromProbability(0.2, 1), 0.0001);
      EXPECT_NEAR(0, Utils::stdDevFromProbability(50, 1), 0.0001);
      EXPECT_NEAR(0.3, Utils::stdDevFromProbability(1, 0.99613), 0.0001);
      EXPECT_NEAR(0.5, Utils::stdDevFromProbability(1, 0.86466), 0.0001);
      EXPECT_NEAR(0.75, Utils::stdDevFromProbability(1, 0.58889), 0.0001);
      EXPECT_NEAR(1, Utils::stdDevFromProbability(1, 0.39347), 0.0001);
      EXPECT_NEAR(1.5, Utils::stdDevFromProbability(1, 0.19926), 0.0001);
      EXPECT_NEAR(2, Utils::stdDevFromProbability(1, 0.11750), 0.0001);
    }

    TEST(stdDevFromProbability, wrongInputValues)
    {
      EXPECT_THROW(Utils::stdDevFromProbability(-1, 0.5), std::range_error);
      EXPECT_THROW(Utils::stdDevFromProbability(0, 0.5), std::range_error);
      EXPECT_THROW(Utils::stdDevFromProbability(1, -0.5), std::range_error);
      EXPECT_THROW(Utils::stdDevFromProbability(1, 1.1), std::range_error);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

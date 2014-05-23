// "Copyright [2014] <owners>"

#include "gtest/gtest.h"

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    TEST(probabilityFromStdDevTest, checkResultsOfLegitValues)
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

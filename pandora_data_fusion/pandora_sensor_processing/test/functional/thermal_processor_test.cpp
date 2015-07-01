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

#include <vector>
#include <string>

#include "gtest/gtest.h"

#include "pandora_common_msgs/GeneralAlert.h"
#include "pandora_common_msgs/GeneralAlertVector.h"

#include "pandora_sensor_processing/clusterer.h"
#include "pandora_sensor_processing/thermal_processor.h"

namespace pandora_sensor_processing
{

  class ThermalProcessorTest : public ::testing::Test
  {
    public:
      ThermalProcessorTest() : thermalProcessor_("/test") {}

    protected:
      void mockCallback(const pandora_common_msgs::GeneralAlertVector& msg)
      {
        responded_ = true;
        incomingAlerts_.push_back(msg);
      }

      virtual void SetUp()
      {
        clusterer_.reset( new Clusterer(16, 3, 100) );
        time_ = 0;
        mockSubscriber_ = nh_->subscribe(
            "/test/alert_output", 1, &ThermalProcessorTest::mockCallback, this);
      }

      virtual void TearDown()
      {
        mockSubscriber_.shutdown();
      }

      static void SetUpTestCase()
      {
        nh_ = new ros::NodeHandle("/test");
        mockPublisher_ = new ros::Publisher;
        *mockPublisher_ = nh_->advertise<sensor_msgs::Image>("/test/raw_input", 1);
      }

      static void TearDownTestCase()
      {
        delete nh_;
        nh_ = NULL;
        delete mockPublisher_;
        mockPublisher_ = NULL;
      }

      /* helper functions */

      /**
       * @brief Makes an image according to the parameters given.
       * If it is to be qualified to an alert, warmer pixels will be to the
       * upper right corner.
       */
      void makeImage(int width, int height,
          std::string frame, bool alert = true)
      {
        if (width < 4)
          width = 4;
        if (height < 4)
          height = 4;
        image_.header.stamp = ros::Time(time_ += 0.5);
        image_.header.frame_id = frame;
        image_.width = width;
        image_.height = height;
        image_.data = std::vector<uint8_t>(width * height);
        for (int ii = 0; ii < width * height; ++ii)
        {
          image_.data[ii] = 25;
          if (alert && ii % width > 1 && ii / width < height - 2)
            image_.data[ii] = 36;
        }
        responded_ = false;
      }

      void publishAndWait()
      {
        mockPublisher_->publish(image_);
        int i = 20;
        while (ros::ok() && i > 0)
        {
          ros::spinOnce();
          ros::Duration d(0.1);
          d.sleep();
          i--;
        }
      }

      /* accessors to private functions */

      bool analyzeImage(const sensor_msgs::Image& msg,
          const ClustererPtr& clusterer)
      {
        return thermalProcessor_.analyzeImage(msg, clusterer);
      }

      bool getResults(const sensor_msgs::Image& msg,
          const ClustererConstPtr& clusterer)
      {
        return thermalProcessor_.getResults(msg, clusterer);
      }

      /* accessors to private variables */

      const FrameToClusterer& getFrameToClusterer()
      {
        return thermalProcessor_.frameToClusterer_;
      }

      /* variables */

      float time_;
      sensor_msgs::Image image_;
      ClustererPtr clusterer_;

      ThermalProcessor thermalProcessor_;

      static ros::NodeHandle* nh_;
      static ros::Publisher* mockPublisher_;
      ros::Subscriber mockSubscriber_;
      std::vector<pandora_common_msgs::GeneralAlertVector> incomingAlerts_;
      bool responded_;
  };

  ros::NodeHandle* ThermalProcessorTest::nh_ = NULL;
  ros::Publisher* ThermalProcessorTest::mockPublisher_ = NULL;

  /* Unit Tests */

  TEST_F(ThermalProcessorTest, analyzeImage_fail_wrong_size)
  {
    makeImage(4, 5, "pur");
    EXPECT_FALSE(analyzeImage(image_, clusterer_));
  }

  TEST_F(ThermalProcessorTest, analyzeImage_ok)
  {
    makeImage(4, 4, "pur");
    EXPECT_EQ(0, clusterer_->getMeasurementsCounter());
    EXPECT_TRUE(analyzeImage(image_, clusterer_));
    EXPECT_EQ(1, clusterer_->getMeasurementsCounter());
  }

  TEST_F(ThermalProcessorTest, getResults_fail_no_alert)
  {
    makeImage(4, 4, "pur", false);
    EXPECT_TRUE(analyzeImage(image_, clusterer_));
    EXPECT_FALSE(getResults(image_, clusterer_));
  }

  TEST_F(ThermalProcessorTest, getResults_ok)
  {
    makeImage(4, 4, "pur");
    analyzeImage(image_, clusterer_);
    EXPECT_TRUE(getResults(image_, clusterer_));
    pandora_common_msgs::GeneralAlert alert = thermalProcessor_.getAlert();
    EXPECT_EQ(image_.header.frame_id, alert.header.frame_id);
    EXPECT_EQ(image_.header.stamp, alert.header.stamp);
    EXPECT_GT(alert.info.probability, 0.3);
  }

  TEST_F(ThermalProcessorTest, getResults_fail_no_alert_in_current)
  {
    makeImage(4, 4, "pur");
    analyzeImage(image_, clusterer_);
    makeImage(4, 4, "pur", false);
    analyzeImage(image_, clusterer_);
    EXPECT_FALSE(getResults(image_, clusterer_));
  }

  TEST_F(ThermalProcessorTest, getResults_ok2)
  {
    makeImage(4, 4, "pur", false);
    analyzeImage(image_, clusterer_);
    makeImage(4, 4, "pur");
    analyzeImage(image_, clusterer_);
    EXPECT_TRUE(getResults(image_, clusterer_));
    pandora_common_msgs::GeneralAlert alert = thermalProcessor_.getAlert();
    EXPECT_EQ(image_.header.frame_id, alert.header.frame_id);
    EXPECT_EQ(image_.header.stamp, alert.header.stamp);
    EXPECT_GT(alert.info.probability, 0.3);
  }

  TEST_F(ThermalProcessorTest, getResults_fail_no_alert_in_current_2)
  {
    makeImage(4, 4, "pur", false);
    analyzeImage(image_, clusterer_);
    makeImage(4, 4, "pur");
    analyzeImage(image_, clusterer_);
    makeImage(4, 4, "pur", false);
    analyzeImage(image_, clusterer_);
    EXPECT_FALSE(getResults(image_, clusterer_));
  }

  /* Functional Tests */

  TEST_F(ThermalProcessorTest, different_frames_create_different_clusterers)
  {
    makeImage(4, 4, "pur", false);
    publishAndWait();
    ASSERT_FALSE(responded_);
    ASSERT_EQ(1, getFrameToClusterer().size());
    EXPECT_NE(getFrameToClusterer().end(), getFrameToClusterer().find("pur"));

    makeImage(4, 4, "ahr", false);
    publishAndWait();
    ASSERT_FALSE(responded_);
    ASSERT_EQ(2, getFrameToClusterer().size());
    EXPECT_NE(getFrameToClusterer().end(), getFrameToClusterer().find("pur"));
    EXPECT_NE(getFrameToClusterer().end(), getFrameToClusterer().find("ahr"));

    makeImage(4, 4, "pur2");
    publishAndWait();
    ASSERT_TRUE(responded_);
    ASSERT_EQ(3, getFrameToClusterer().size());
    EXPECT_NE(getFrameToClusterer().end(), getFrameToClusterer().find("pur"));
    EXPECT_NE(getFrameToClusterer().end(), getFrameToClusterer().find("pur2"));
    EXPECT_NE(getFrameToClusterer().end(), getFrameToClusterer().find("ahr"));
  }

  TEST_F(ThermalProcessorTest, correct_responses)
  {
    // This is cool because raw measurement is to be qualified to an alert.
    makeImage(5, 5, "pur");
    publishAndWait();
    ASSERT_TRUE(responded_);
    ASSERT_EQ(1, incomingAlerts_.size());
    EXPECT_EQ("pur", incomingAlerts_.at(0).header.frame_id);
    EXPECT_FLOAT_EQ(0.39894228, incomingAlerts_.at(0).alerts[0].probability);

    // This in not cool because image is not the same size as the other images
    // of frame "pur"
    makeImage(8, 8, "pur");
    publishAndWait();
    ASSERT_FALSE(responded_);

    // This is not cool because this image does not contain info that can make
    // an alert.
    makeImage(5, 5, "pur", false);
    publishAndWait();
    ASSERT_FALSE(responded_);

    // Failing or not qualifying images do not interfere with the processor.
    // This image shall qualify to an alert.
    makeImage(5, 5, "pur");
    publishAndWait();
    ASSERT_TRUE(responded_);
    ASSERT_EQ(2, incomingAlerts_.size());
  }

}  // namespace pandora_sensor_processing

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "thermal_processor_test");

  int result = RUN_ALL_TESTS();
  return result;
}


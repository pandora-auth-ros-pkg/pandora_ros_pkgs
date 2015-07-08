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
* Author: Vassilis Choutas
*********************************************************************/

#include <vector>
#include <string>

#include <ros/package.h>

#include <gtest/gtest.h>

#include "pandora_vision_victim/feature_extractors/sift_extractor.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  class SiftExtractorTest : public ::testing::Test
  {
  public:
    SiftExtractorTest()
    {
    }

    virtual void SetUp()
    {
      std::string packagePath = ros::package::getPath("pandora_vision_victim");
      testImg = cv::imread(packagePath + "/test/unit/test_data/box.png");

      ASSERT_TRUE(testImg.data != NULL) << "Could not read the test image!";
    }

    virtual ~SiftExtractorTest()
    {
    }

    virtual void TearDown()
    {
      delete siftExtractorTestFixture_;
    }

    /*
     * @brief : Creates a keypoint detector.
     * @param detectorName[const std::string&]: The type of the keypoint
     *  detector we wish to create.
     * @return : The pointer to the keypoint detector.
     */
    cv::Ptr<cv::FeatureDetector> create_detector(const std::string&
                                                 detectorName)
    {
      return cv::FeatureDetector::create(detectorName);
    }

    /*
     * @brief : Creates a descriptor extractor.
     * @param extractorName[const std::string&]: The type of the descriptor
     *  extractor we wish to create.
     * @return : The pointer to the descriptor extractor.
     */
    cv::Ptr<cv::DescriptorExtractor> create_extractor(
        const std::string& extractorName)
    {
      return cv::DescriptorExtractor::create(extractorName);
    }

    /// The image used for testingg purposes.
    cv::Mat testImg;

    /// A pointer to the extractor siftExtractorTestFixture_ that will be tested.
    SiftExtractor* siftExtractorTestFixture_;

    /// The pointer the keypoint detector that produces the correct results.
    cv::Ptr<cv::FeatureDetector> detector;
    /// The pointer to the feature extractor that produces the correct
    /// results.
    cv::Ptr<cv::DescriptorExtractor> extractor;
  };


  TEST_F(SiftExtractorTest, SiftTest)
  {
    // Create the SIFT siftExtractorTestFixture_
    siftExtractorTestFixture_ = new SiftExtractor("SIFT", "SIFT");
    ASSERT_TRUE(siftExtractorTestFixture_ != NULL) << "Could not Feature Extractor Object!";

    // Create the SIFT keypoint detector.
    detector = create_detector("SIFT");
    ASSERT_TRUE(detector != NULL) << "Could not create SIFT keypoint" <<
      " detector!";
    // Create the SIFT feature extractor.
    extractor = create_extractor("SIFT");
    ASSERT_TRUE(extractor != NULL) << "Could not create SIFT feature"
      << " extractor!";

    cv::Mat grayImg;
    // Check that the image was read successfully.
    // Convert the image to gray scale.
    if (testImg.channels() > 1)
      cv::cvtColor(testImg, grayImg, CV_BGR2GRAY);
    else
      grayImg = testImg;

    std::vector<cv::KeyPoint> trueKeypoints;
    cv::Mat trueDescriptors;

    detector->detect(grayImg, trueKeypoints);

    extractor->compute(grayImg, trueKeypoints, trueDescriptors);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    siftExtractorTestFixture_->extractFeatures(grayImg, &descriptors);

    ASSERT_EQ(trueDescriptors.rows, descriptors.rows);
    ASSERT_EQ(trueDescriptors.cols, descriptors.cols);

    for (int i = 0; i < trueDescriptors.cols; ++i)
    {
      for (int j = 0; j < trueDescriptors.rows; ++j)
      {
        ASSERT_FLOAT_EQ(trueDescriptors.at<float>(i, j),
                        descriptors.at<float>(i, j));
      }
    }
  }  // End of SiftTest

  TEST_F(SiftExtractorTest, DenseSiftTest)
  {
    siftExtractorTestFixture_ = new SiftExtractor("Dense", "SIFT");
    ASSERT_TRUE(siftExtractorTestFixture_ != NULL) << "Could not Feature Extractor Object!";

    detector = create_detector("Dense");
    ASSERT_TRUE(detector != NULL) << "Could not create SIFT keypoint" <<
      " detector!";
    extractor = create_extractor("SIFT");
    ASSERT_TRUE(extractor != NULL) << "Could not create SIFT feature"
      << " extractor!";

    cv::Mat grayImg;

    // Convert the image to gray scale.
    if (testImg.channels() > 1)
      cv::cvtColor(testImg, grayImg, CV_BGR2GRAY);
    else
      grayImg = testImg;

    std::vector<cv::KeyPoint> trueKeypoints;
    cv::Mat trueDescriptors;

    detector->detect(grayImg, trueKeypoints);

    extractor->compute(grayImg, trueKeypoints, trueDescriptors);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    siftExtractorTestFixture_->extractFeatures(grayImg, &descriptors);

    ASSERT_EQ(trueDescriptors.rows, descriptors.rows);
    ASSERT_EQ(trueDescriptors.cols, descriptors.cols);

    for (int i = 0; i < trueDescriptors.cols; ++i)
    {
      for (int j = 0; j < trueDescriptors.rows; ++j)
      {
        ASSERT_FLOAT_EQ(trueDescriptors.at<float>(i, j),
                        descriptors.at<float>(i, j));
      }
    }
  }  // End of DenseSiftTest

}   // namespace pandora_vision_victim
}  // namespace pandora_vision

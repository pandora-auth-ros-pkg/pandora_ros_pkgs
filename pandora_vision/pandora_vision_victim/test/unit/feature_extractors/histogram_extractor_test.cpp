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

#include <unistd.h>

#include <vector>
#include <map>
#include <algorithm>
#include <utility>
#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>

#include "pandora_vision_victim/feature_extractors/histogram_extractor.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  class HistogramExtractorTest: public ::testing::Test
  {
    public:
      HistogramExtractorTest()
      {
      }

      virtual void SetUp()
      {
        width_ = 10;
        height_ = 10;
        float bgrRange[] = {0, 256, 0, 256, 0, 256};
        // Initialize the ranges for the BGR color space.
        colorRanges_["bgr"] = std::vector<float>(bgrRange, bgrRange + 6);
        // Initialize the ranges for the BGR color space.
        float hsvRange[] = {0, 180, 0, 256, 0, 256};
        colorRanges_["hsv"] = std::vector<float>(hsvRange, hsvRange + 6);
        // Initialize the ranges for the BGR color space.
        float yCrCbRange[] = {0, 256, 0, 256, 0, 256};
        colorRanges_["yCrCb"] = std::vector<float>(yCrCbRange, yCrCbRange + 6);
        // Initialize the ranges for the BGR color space.
        float labRange[] = {0, 256, 0, 256, 0, 256};
        colorRanges_["lab"] = std::vector<float>(labRange, labRange + 6);

        // Get all the possible channel permutations.
        std::vector<int> permutation;
        permutation.push_back(0);
        channelPermutations_.push_back(permutation);

        permutation[0] = 1;
        channelPermutations_.push_back(permutation);

        permutation[0] = 2;
        channelPermutations_.push_back(permutation);

        // Store all the possible 2-permutations of the image channels.
        permutation[0] = 0;
        permutation.push_back(1);
        channelPermutations_.push_back(permutation);

        permutation[1] = 2;
        channelPermutations_.push_back(permutation);

        permutation[0] = 1;
        channelPermutations_.push_back(permutation);

        int myints[] = {0, 1, 2};
        std::sort(myints, myints + 3);

        // Store all the possible 3-permutations of the image channels.
        channelPermutations_.push_back(std::vector<int>(myints, myints + 3));

        while (std::next_permutation(myints, myints + 3))
          channelPermutations_.push_back(std::vector<int>(myints, myints + 3));
      }

      virtual ~HistogramExtractorTest()
      {
      }

      virtual void TearDown()
      {
        delete histExtractorTestFixturePtr_;
      }

      void createRangeCombinations(
          std::vector<std::vector<float> >* outputRanges,
          const std::vector<int>& permutation,
          const std::vector<int>& binSizes,
          const std::vector<float>& maxRanges)
      {
        outputRanges->clear();
        if (permutation.size() == 1)
        {
          int step = maxRanges[1] / binSizes[0];
          for (int k = 0; k < binSizes[0]; ++k)
          {
            std::vector<float> ranges(6);
            ranges[permutation[0] * 2] = k * step;
            ranges[permutation[0] * 2 + 1] = (k + 1) * step - 1;
            outputRanges->push_back(ranges);
          }
        }
        else if (permutation.size() == 2)
        {
          int step1 = maxRanges[1] / binSizes[0];
          int step2 = maxRanges[3] / binSizes[1];
          for (int k = 0; k < binSizes[0]; ++k)
          {
            std::vector<float> ranges(6);
            ranges[permutation[0] * 2] = k * step1;
            ranges[permutation[0] * 2 + 1] = (k + 1) * step1 - 1;
            for (int j = 0; j < binSizes[1]; ++j)
            {
              ranges[permutation[1] * 2] = j * step2;
              ranges[permutation[1] * 2 + 1] = (j + 1) * step2 - 1;
              outputRanges->push_back(ranges);
            }
          }
        }
        else if (permutation.size() == 3)
        {
          int step1 = maxRanges[1] / binSizes[0];
          int step2 = maxRanges[3] / binSizes[1];
          int step3 = maxRanges[5] / binSizes[2];

          for (int k = 0; k < binSizes[0]; ++k)
          {
            std::vector<float> ranges(6);
            ranges[permutation[0] * 2] = k * step1;
            ranges[permutation[0] * 2 + 1] = (k + 1) * step1 - 1;
            for (int j = 0; j < binSizes[1]; ++j )
            {
              ranges[permutation[1] * 2] = j * step2;
              ranges[permutation[1] * 2 + 1] = (j + 1) * step2 - 1;
              for (int ii = 0; ii < binSizes[2]; ++ii)
              {
                ranges[permutation[2] * 2] = ii * step3;
                ranges[permutation[2] * 2 + 1] = (ii + 1) * step3 - 1;
                outputRanges->push_back(ranges);
              }
            }
          }
        }
      }

      void createSingleChannelImage(cv::Mat* outputImage,
          const std::pair<float, float>& range,
          int width_, int height_)
      {
        *outputImage = cv::Mat::zeros(width_, height_, CV_8UC1);
        // outputImage->setTo(static_cast<short unsigned int>(
              // (range.first + range.second) / 2));
        outputImage->setTo(static_cast<int>(range.first + 1));
      }


      /**
       * @brief: Creates a multi channel image in the RGB color Space
       * with values for each channel in the corresponding range and the
       * given size.
       */
      void createMultiChannelImage(cv::Mat* outputImage,
          std::vector<bool> validChannel,
          std::vector<float> ranges,
          int width, int height)
      {
        std::vector<cv::Mat> channels;
        for (int i = 0; i < validChannel.size(); ++i)
        {
          cv::Mat tempImage;
          if (validChannel[i])
          {
            std::pair<float, float> range(ranges[2 * i], ranges[2 * i + 1]);
            createSingleChannelImage(&tempImage, range, width, height);
          }
          else
            tempImage = cv::Mat::zeros(height, width, CV_8UC1);
          channels.push_back(tempImage);
        }
        cv::merge(channels, *outputImage);
      }

      /// The width_ of the test images.
      int width_;
      /// The height_ of the test images.
      int height_;
      std::vector<std::vector<int> > channelPermutations_;
      std::map<std::string, std::vector<float> > colorRanges_;
      HistogramExtractor* histExtractorTestFixturePtr_;
  };

  TEST_F(HistogramExtractorTest, BGRHistTest)
  {
    cv::Mat testImage;
    int bins[] = {32, 32, 32};
    std::vector<int> histBins;
    std::vector<float> histRanges;

    // Iterate over all the possible channel combinations.
    for (int i = 0; i < channelPermutations_.size(); ++i)
    {
      // For each channel in the current configuration:
      for (int j = 0; j < channelPermutations_[i].size(); j++)
      {
        // Store the number of histogram bins that correspond to the current
        // permutation of the image channels.
        histBins.push_back(bins[channelPermutations_[i][j]]);

        // Do the same for the range of values of each channel of the
        // current color space. The resulting vector has size equal to two
        // times the number of channels we want for our histogram. The even
        // values are the lower bounds and the odd values are the upper bounds
        // for each channel.
        histRanges.push_back(colorRanges_["bgr"][channelPermutations_[i][j]
            * 2]);
        histRanges.push_back(colorRanges_["bgr"][channelPermutations_[i][j] * 2
            + 1]);
      }

      // Create the correct extractor for this configuration.
      histExtractorTestFixturePtr_ = new HistogramExtractor(
          channelPermutations_[i],
          1, histBins, histRanges,
          std::string("BGR"), false);

      // Check that the histogram feature extractor has been created.
      ASSERT_FALSE(histExtractorTestFixturePtr_ == NULL) << "Could not create"
        << " Color Histogram Feature extractor object!" << std::endl;

      std::vector<bool> validChannels(3);
      // Set the valid channels flag to add values only to the channels that
      // will be tested in this iteration.
      // If only one channel is tested then set the corresponding value.
      if (channelPermutations_[i].size() == 1)
        validChannels[channelPermutations_[i][0]] = true;
      else
        // Set all the necessary channel flags.
        for (int j = 0; j < channelPermutations_[i].size(); ++j)
          validChannels[channelPermutations_[i][j]] = true;

      // Create all possible combinations for the histogram range values
      // for the current channel permutation.
      std::vector< std::vector<float> > rangeCombinations;

      createRangeCombinations(&rangeCombinations, channelPermutations_[i],
          histBins, histRanges);

      // Calculate the true feature vector size.
      int featureVectorExpectedSize = 0;
      for (int ii = 0; ii < histBins.size(); ++ii)
        featureVectorExpectedSize += histBins[ii];

      cv::Mat features;
      // Iterate over all the above created values.
      for (int ii = 0; ii < rangeCombinations.size(); ++ii)
      {
        // Create a new test image by assigning non-zero values only to the
        // designated channels in the provided ranges.
        createMultiChannelImage(&testImage, validChannels,
            rangeCombinations[ii],
            width_, height_);
        histExtractorTestFixturePtr_->extractFeatures(testImage, &features);

        // Check that the returned vector has the size we expect.
        ASSERT_EQ(featureVectorExpectedSize, features.rows) << "The expected"
          << " size of the feature vector is not equal to the one returned"
          " by the feature extraction function!";

        int offset = 0;
        int pos;
        // For each channel of the image that will be included in the
        // histogram:
        for (int j = 0; j < channelPermutations_[i].size(); ++j)
        {
          // Check if it is included.
          if (validChannels[channelPermutations_[i][j]] == true)
          {
            // Get the position of the non-zero values for the histogram
            // by finding the bin that corresponds to the values we assigned
            // during the image creation.
            pos = rangeCombinations[ii][2 *  channelPermutations_[i][j]] /
              histRanges[2 * j + 1] * histBins[j];
            // Add an offset so as to get to the correct block of the feature
            // vector(the block that corresponds to the current channel).
            pos = pos + offset;
            ASSERT_FLOAT_EQ(features.at<float>(pos), 1.0f) << "The value of "
              "the feature vector is not equal to the expected one at "
              << "position : " << pos << " of the feature vector!";
          }
          // Update the offset by the number of bins.
          offset += histBins[j];
        }
      }

      // Clear the vectors so as to use them to set up the test for
      // the next iteration.
      histBins.clear();
      histRanges.clear();
      validChannels.clear();
    }
  }  // End of BGR color histogram extractor test.

  TEST_F(HistogramExtractorTest, HSVHistTest)
  {
    cv::Mat testImage;
    int bins[] = {30, 32, 32};
    std::vector<int> histBins;
    std::vector<float> histRanges;

    // Iterate over all the possible channel combinations.
    for (int i = 0; i < channelPermutations_.size(); ++i)
    {
      // For each channel in the current configuration:
      for (int j = 0; j < channelPermutations_[i].size(); j++)
      {
        // Store the number of histogram bins that correspond to the current
        // permutation of the image channels.
        histBins.push_back(bins[channelPermutations_[i][j]]);

        // Do the same for the range of values of each channel of the
        // current color space. The resulting vector has size equal to two
        // times the number of channels we want for our histogram. The even
        // values are the lower bounds and the odd values are the upper bounds
        // for each channel.
        histRanges.push_back(colorRanges_["hsv"][channelPermutations_[i][j]
            * 2]);
        histRanges.push_back(colorRanges_["hsv"][channelPermutations_[i][j] * 2
            + 1]);
      }

      // Create the correct extractor for this configuration.
      histExtractorTestFixturePtr_ = new HistogramExtractor(
          channelPermutations_[i],
          1, histBins, histRanges,
          std::string(""), false);

      // Check that the histogram feature extractor has been created.
      ASSERT_FALSE(histExtractorTestFixturePtr_ == NULL) << "Could not create"
        << " Color Histogram Feature extractor object!" << std::endl;

      std::vector<bool> validChannels(3);
      // Set the valid channels flag to add values only to the channels that
      // will be tested in this iteration.
      // If only one channel is tested then set the corresponding value.
      int count = 0;
      if (channelPermutations_[i].size() == 1)
        validChannels[channelPermutations_[i][0]] = true;
      else
        // Set all the necessary channel flags.
        for (int j = 0; j < channelPermutations_[i].size(); ++j)
        {
          validChannels[channelPermutations_[i][j]] = true;
          count++;
        }

      // Create all possible combinations for the histogram range values
      // for the current channel permutation.
      std::vector< std::vector<float> > rangeCombinations;

      createRangeCombinations(&rangeCombinations, channelPermutations_[i],
          histBins, histRanges);

      // Calculate the true feature vector size.
      int featureVectorExpectedSize = 0;
      for (int ii = 0; ii < histBins.size(); ++ii)
        featureVectorExpectedSize += histBins[ii];

      cv::Mat features;
      // Iterate over all the above created values.
      for (int ii = 0; ii < rangeCombinations.size(); ++ii)
      {
        // Create a new test image by assigning non-zero values only to the
        // designated channels in the provided ranges.
        createMultiChannelImage(&testImage, validChannels,
            rangeCombinations[ii],
            width_, height_);

        histExtractorTestFixturePtr_->extractFeatures(testImage, &features);

        // Check that the returned vector has the size we expect.
        ASSERT_EQ(featureVectorExpectedSize, features.rows) << "The expected"
          << " size of the feature vector is not equal to the one returned"
          " by the feature extraction function!";

        int offset = 0;
        int pos;
        // For each channel of the image that will be included in the
        // histogram:
        for (int j = 0; j < channelPermutations_[i].size(); ++j)
        {
          // Check if it is included.
          if (validChannels[j] == true)
          {
            // Get the position of the non-zero values for the histogram
            // by finding the bin that corresponds to the values we assigned
            // during the image creation.
            pos = rangeCombinations[ii][2 *  channelPermutations_[i][j] + 1] /
              (histRanges[2 * j + 1] / histBins[j]);
            // Add an offset so as to get to the correct block of the feature
            // vector(the block that corresponds to the current channel).
            pos = pos + offset;
            ASSERT_FLOAT_EQ(features.at<float>(pos), 1.0f) << "The value of "
              "the feature vector is not equal to the expected one at "
              << "position : " << pos << " of the feature vector!";
          }
          // Update the offset by the number of bins.
          offset += histBins[j];
        }
      }

      // Clear the vectors so as to use them to set up the test for
      // the next iteration.
      histBins.clear();
      histRanges.clear();
      validChannels.clear();
    }
  }  // End of HSV color histogram extractor test.

  TEST_F(HistogramExtractorTest, YCrCbHistTest)
  {
    cv::Mat testImage;
    int bins[] = {32, 32, 32};
    std::vector<int> histBins;
    std::vector<float> histRanges;

    // Iterate over all the possible channel combinations.
    for (int i = 0; i < channelPermutations_.size(); ++i)
    {
      // For each channel in the current configuration:
      for (int j = 0; j < channelPermutations_[i].size(); j++)
      {
        // Store the number of histogram bins that correspond to the current
        // permutation of the image channels.
        histBins.push_back(bins[channelPermutations_[i][j]]);

        // Do the same for the range of values of each channel of the
        // current color space. The resulting vector has size equal to two
        // times the number of channels we want for our histogram. The even
        // values are the lower bounds and the odd values are the upper bounds
        // for each channel.
        histRanges.push_back(colorRanges_["yCrCb"][channelPermutations_[i][j]
            * 2]);
        histRanges.push_back(colorRanges_["yCrCb"][channelPermutations_[i][j] * 2
            + 1]);
      }

      // Create the correct extractor for this configuration.
      histExtractorTestFixturePtr_ = new HistogramExtractor(
          channelPermutations_[i],
          1, histBins, histRanges,
          std::string(""), false);

      // Check that the histogram feature extractor has been created.
      ASSERT_FALSE(histExtractorTestFixturePtr_ == NULL) << "Could not create"
        << " Color Histogram Feature extractor object!" << std::endl;

      std::vector<bool> validChannels(3);
      // Set the valid channels flag to add values only to the channels that
      // will be tested in this iteration.
      // If only one channel is tested then set the corresponding value.
      if (channelPermutations_[i].size() == 1)
        validChannels[channelPermutations_[i][0]] = true;
      else
        // Set all the necessary channel flags.
        for (int j = 0; j < channelPermutations_[i].size(); ++j)
          validChannels[channelPermutations_[i][j]] = true;

      // Create all possible combinations for the histogram range values
      // for the current channel permutation.
      std::vector< std::vector<float> > rangeCombinations;

      createRangeCombinations(&rangeCombinations, channelPermutations_[i],
          histBins, histRanges);

      // Calculate the true feature vector size.
      int featureVectorExpectedSize = 0;
      for (int ii = 0; ii < histBins.size(); ++ii)
        featureVectorExpectedSize += histBins[ii];

      cv::Mat features;
      // Iterate over all the above created values.
      for (int ii = 0; ii < rangeCombinations.size(); ++ii)
      {
        // Create a new test image by assigning non-zero values only to the
        // designated channels in the provided ranges.
        createMultiChannelImage(&testImage, validChannels,
            rangeCombinations[ii],
            width_, height_);

        histExtractorTestFixturePtr_->extractFeatures(testImage, &features);

        // Check that the returned vector has the size we expect.
        ASSERT_EQ(featureVectorExpectedSize, features.rows) << "The expected"
          << " size of the feature vector is not equal to the one returned"
          " by the feature extraction function!";

        int offset = 0;
        int pos;
        // For each channel of the image that will be included in the
        // histogram:
        for (int j = 0; j < channelPermutations_[i].size(); ++j)
        {
          // Check if it is included.
          if (validChannels[channelPermutations_[i][j]] == true)
          {
            // Get the position of the non-zero values for the histogram
            // by finding the bin that corresponds to the values we assigned
            // during the image creation.
            pos = rangeCombinations[ii][2 *  channelPermutations_[i][j]] /
              histRanges[2 * j + 1] * histBins[j];
            // Add an offset so as to get to the correct block of the feature
            // vector(the block that corresponds to the current channel).
            pos = pos + offset;
            ASSERT_FLOAT_EQ(features.at<float>(pos), 1.0f) << "The value of "
              "the feature vector is not equal to the expected one at "
              << "position : " << pos << " of the feature vector!";
          }
          // Update the offset by the number of bins.
          offset += histBins[j];
        }
      }

      // Clear the vectors so as to use them to set up the test for
      // the next iteration.
      histBins.clear();
      histRanges.clear();
      validChannels.clear();
    }
  }  // End of YCrCb color histogram extractor test.

  TEST_F(HistogramExtractorTest, CIELabHistTest)
  {
    cv::Mat testImage;
    int bins[] = {32, 32, 32};
    std::vector<int> histBins;
    std::vector<float> histRanges;

    // Iterate over all the possible channel combinations.
    for (int i = 0; i < channelPermutations_.size(); ++i)
    {
      // For each channel in the current configuration:
      for (int j = 0; j < channelPermutations_[i].size(); j++)
      {
        // Store the number of histogram bins that correspond to the current
        // permutation of the image channels.
        histBins.push_back(bins[channelPermutations_[i][j]]);

        // Do the same for the range of values of each channel of the
        // current color space. The resulting vector has size equal to two
        // times the number of channels we want for our histogram. The even
        // values are the lower bounds and the odd values are the upper bounds
        // for each channel.
        histRanges.push_back(colorRanges_["lab"][channelPermutations_[i][j]
            * 2]);
        histRanges.push_back(colorRanges_["lab"][channelPermutations_[i][j] * 2
            + 1]);
      }

      // Create the correct extractor for this configuration.
      histExtractorTestFixturePtr_ = new HistogramExtractor(
          channelPermutations_[i],
          1, histBins, histRanges,
          std::string(""), false);

      // Check that the histogram feature extractor has been created.
      ASSERT_FALSE(histExtractorTestFixturePtr_ == NULL) << "Could not create"
        << " Color Histogram Feature extractor object!" << std::endl;

      std::vector<bool> validChannels(3);
      // Set the valid channels flag to add values only to the channels that
      // will be tested in this iteration.
      // If only one channel is tested then set the corresponding value.
      if (channelPermutations_[i].size() == 1)
        validChannels[channelPermutations_[i][0]] = true;
      else
        // Set all the necessary channel flags.
        for (int j = 0; j < channelPermutations_[i].size(); ++j)
          validChannels[channelPermutations_[i][j]] = true;

      // Create all possible combinations for the histogram range values
      // for the current channel permutation.
      std::vector< std::vector<float> > rangeCombinations;

      createRangeCombinations(&rangeCombinations, channelPermutations_[i],
          histBins, histRanges);

      // Calculate the true feature vector size.
      int featureVectorExpectedSize = 0;
      for (int ii = 0; ii < histBins.size(); ++ii)
        featureVectorExpectedSize += histBins[ii];

      cv::Mat features;
      // Iterate over all the above created values.
      for (int ii = 0; ii < rangeCombinations.size(); ++ii)
      {
        // Create a new test image by assigning non-zero values only to the
        // designated channels in the provided ranges.
        createMultiChannelImage(&testImage, validChannels,
            rangeCombinations[ii],
            width_, height_);
        histExtractorTestFixturePtr_->extractFeatures(testImage, &features);

        // Check that the returned vector has the size we expect.
        ASSERT_EQ(featureVectorExpectedSize, features.rows) << "The expected"
          << " size of the feature vector is not equal to the one returned"
          " by the feature extraction function!";

        int offset = 0;
        int pos;
        // For each channel of the image that will be included in the
        // histogram:
        for (int j = 0; j < channelPermutations_[i].size(); ++j)
        {
          // Check if it is included.
          if (validChannels[channelPermutations_[i][j]] == true)
          {
            // Get the position of the non-zero values for the histogram
            // by finding the bin that corresponds to the values we assigned
            // during the image creation.
            pos = rangeCombinations[ii][2 *  channelPermutations_[i][j]] /
              histRanges[2 * j + 1] * histBins[j];
            // Add an offset so as to get to the correct block of the feature
            // vector(the block that corresponds to the current channel)
            pos = pos + offset;
            ASSERT_FLOAT_EQ(features.at<float>(pos), 1.0f) << "The value of "
              "the feature vector is not equal to the expected one at "
              << "position : " << pos << " of the feature vector!";
          }
          // Update the offset by the number of bins.
          offset += histBins[j];
        }
      }

      // Clear the vectors so as to use them to set up the test for
      // the next iteration.
      histBins.clear();
      histRanges.clear();
      validChannels.clear();
    }
  }  // End of CIELab color histogram extractor test.

}  // namespace pandora_vision_victim
}  // namespace pandora_vision

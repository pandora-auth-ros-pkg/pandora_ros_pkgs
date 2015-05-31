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
 * Author: Alexandros Philotheou, Vasilis Bosdelekidis
 *********************************************************************/

#include "utils/visualization.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class VisualizationTest
    @brief Tests the integrity of methods of class Visualization
   **/
  class VisualizationTest : public ::testing::Test
  {
    protected:

      VisualizationTest () {}

      virtual void SetUp()
      {
        HEIGHT = 480;
        WIDTH = 640;

        // Construct image floats
        floats = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        for ( int rows = 100; rows < 200; rows++ )
        {
          for ( int cols = 100; cols < 200; cols++ )
          {
            floats.at< float >( rows, cols ) = 2.0;
          }
        }

        // Construct image gray
        gray = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        for ( int rows = 100; rows < 200; rows++ )
        {
          for ( int cols = 100; cols < 200; cols++ )
          {
            gray.at< unsigned char >( rows, cols ) = 255;
          }
        }

        // Construct image rgb
        rgb = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

        for ( int rows = 100; rows < 200; rows++ )
        {
          for ( int cols = 100; cols < 200; cols++ )
          {
            rgb.at< cv::Vec3b >( rows, cols ).val[0] = 255;
          }
        }

        // Set up the conveyor

        cv::Point2f k ( 150, 150 );
        conveyor.keypoint.push_back(k);
        cv::Rect r(100, 100, 99, 99);
        conveyor.rectangle.push_back(r);
      }

      // The images' dimensions
      int HEIGHT;
      int WIDTH;

      // An image containing floats
      cv::Mat floats;

      // A grayscale image
      cv::Mat gray;

      // A RGB image
      cv::Mat rgb;

      // A dummy conveyor
      HolesConveyor conveyor;

  };



  //! Tests Visualization::multipleShow
  TEST_F ( VisualizationTest, multipleShowTest )
  {
    // The input vector of images to show
    std::vector< cv::Mat > images;

    // Push back the images into the vector
    images.push_back( gray );
    images.push_back( rgb );

    ASSERT_EQ ( 2, images.size() );

    // The vector of titles
    std::vector<std::string> titles;

    // Push back the title of each image into the vector
    titles.push_back( "Image gray" );
    titles.push_back( "Image rgb" );

    ASSERT_EQ ( 2, titles.size() );

    // Run Visualization::multipleShow
    // Uncomment for visual inspection
    //Visualization::multipleShow( "Overall title", images, titles, 1000, 0 );
  }



  //! Tests Visualization::scaleImageForVisualization
  TEST_F ( VisualizationTest, scaleImageForVisualizationTest )
  {
    for ( int m = 0; m < 3; m++ )
    {
      // Run Visualization::scaleImageForVisualization
      cv::Mat ret = Visualization::scaleImageForVisualization( floats, m );

      EXPECT_EQ ( CV_8UC1, ret.type() );
    }
  }



  //! Tests Visualization::show
  TEST_F ( VisualizationTest, showTest )
  {
    // Run Visualization::show
    // Uncomment for visual inspection
    //Visualization::show( "Floats image", floats, 0 );
    //Visualization::show( "Gray image", gray, 0 );
    //Visualization::show( "Rgb image", rgb, 0 );
  }



  //! Tests Visualization::showHoles
  TEST_F ( VisualizationTest, showHolesTest )
  {
    // The msgs vector
    std::vector<std::string> msgs;
    msgs.push_back( "gray" );

    // Run Visualization::showHoles
    // Uncomment for visual inspection
    //Visualization::showHoles( "Overall title", gray, conveyor, 0, msgs, 1 );


    msgs.clear();
    msgs.push_back( "rgb" );

    // Run Visualization::showHoles
    // Uncomment for visual inspection
    //Visualization::showHoles( "Overall title", rgb, conveyor, 0, msgs, 1 );
  }



  //! Tests Visualization::showKeypoints
  TEST_F ( VisualizationTest, showKeypointsTest )
  {
    // The vector of keypoints
    std::vector< cv::Point2f > kVector;

    cv::Point2f k ( 150, 150 );
    kVector.push_back( k );

    // The vector of available images
    std::vector< cv::Mat > images;

    images.push_back( floats );
    images.push_back( gray );
    images.push_back( rgb );

    for ( int i = 0; i < 3; i++ )
    {
      cv::Mat ret =
        Visualization::showKeypoints( "Title", images[i], -1, kVector );

      ASSERT_EQ ( CV_8U, ret.depth() );
    }
  }



  //! Tests Visualization::showScaled
  TEST_F ( VisualizationTest, showScaledTest )
  {
    // Uncomment for visual inspection
    //Visualization::showScaled( "Title", floats, 0 );
  }

} // namespace pandora_vision

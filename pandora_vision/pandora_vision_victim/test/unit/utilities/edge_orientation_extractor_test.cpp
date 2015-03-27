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
* Author: Marios Protopapas
*********************************************************************/

#include "pandora_vision_victim/utilities/edge_orientation_extractor.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class EdgeOrientationExtractorTest 
    @brief Tests the integrity of methods of class EdgeOrientationExtractor
   **/
  class EdgeOrientationExtractorTest : public ::testing::Test
  {
    protected:
      

      EdgeOrientationExtractorTest () {}

      
       //! Sets up images needed for testing
       virtual void SetUp()
      {

       HEIGHT = 480;
       WIDTH = 640;

        // Construct a black image
        black = cv::Mat::zeros(HEIGHT,WIDTH, CV_8U);
        
       	// Construct a white image
        white = cv::Mat::zeros(HEIGHT,WIDTH, CV_8U);
        cv::Rect rect(0, 0, WIDTH, HEIGHT);
	cv::rectangle(white, rect, cv::Scalar(255,0,0), -1);
        

	// Construct a horizontal edge image
        horizontal = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U); 
        cv::Rect rect1(0, 0, WIDTH, HEIGHT/2);
	cv::rectangle(horizontal, rect1, cv::Scalar(255,0,0), -1);
        
	// Construct a vertical edge image
        vertical = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U); 
        cv::Rect rect2(0, 0, WIDTH/2, HEIGHT);
	cv::rectangle(vertical, rect2, cv::Scalar(255,0,0), -1);
	
        // Construct a 135-degree diagonal edge image
        diagonal45 = cv::Mat::zeros( 480, 480, CV_8U); 
        cv::Point triangle_points[1][3];
        triangle_points[0][0] = cv::Point(0, 0);
        triangle_points[0][1] = cv::Point(479, 0);
        triangle_points[0][2] = cv::Point(0, 479);
        const cv::Point* ppt[1] = { triangle_points[0] };
        int npt[] = { 3 };
        cv::fillPoly(diagonal45, ppt, npt, 1, cv::Scalar(255, 255, 255), 8); 

 
        // Construct a 45-degree diagonal edge image
        diagonal135 = cv::Mat::zeros( 480, 480, CV_8U); 
        triangle_points[0][0] = cv::Point(0, 0);
        triangle_points[0][1] = cv::Point(479, 0);
        triangle_points[0][2] = cv::Point(479, 479);
        const cv::Point* ppt2[1] = { triangle_points[0] };
        cv::fillPoly(diagonal135, ppt2, npt, 1, cv::Scalar(255, 255, 255), 8); 

        //Construct an image with a circle
        circle = cv::Mat::zeros(HEIGHT, WIDTH, CV_8U);
        cv::Point center = cv::Point(WIDTH/2,HEIGHT/2);
        cv::circle(circle, center, 100, cv::Scalar(255,255,255), -1, 8);


        //Construct a noisy image
        noisy = cv::Mat::zeros(HEIGHT, WIDTH, CV_8U);
        cv::randn(noisy, 128, 30);
/*        cv::imshow("noisy", noisy);*/
        /*cv::waitKey(0);*/

      }
            
      //Image Dimensions
      int HEIGHT, WIDTH;  

      // Images that will be used for testing
      cv::Mat black, white, horizontal, vertical, diagonal45, diagonal135, circle, noisy;
  
  };
  
  
  //! Tests EdgeOrientationExtractor::findEdgeFeatures

  TEST_F ( EdgeOrientationExtractorTest, NoEdgesEmptyImage)
  {
    // The output vector
    std::vector<double>  out;
    EdgeOrientationExtractor::findEdgeFeatures(black, &out);
    ASSERT_EQ ( 80, out.size() ); 
    
    bool flag = true;
    for (int ii = 0; ii < out.size(); ii++)
	    if(out[ii] != 0)
		    flag = false;
    /*
     *ROS_INFO_STREAM("EdgeFeatures= ");
     *for (int ii = 0; ii < out.size(); ii++)
     *          ROS_INFO_STREAM( " " << out[ii]);
     */

    EXPECT_EQ ( true , flag );
  }
  TEST_F ( EdgeOrientationExtractorTest, NoEdgesfilledImage)
  {
    // The output vector
    std::vector<double>  out;
    EdgeOrientationExtractor::findEdgeFeatures(white, &out);
    ASSERT_EQ ( 80, out.size() ); 
    
    bool flag = true;
    for (int ii = 0; ii < out.size(); ii++)
	    if(out[ii] != 0)
		    flag = false;
    //uncomment to visualize
    
/*
 *    cv::imshow("white", white);
 *    cv::waitKey(0);
 *    ROS_INFO_STREAM("EdgeFeatures= ");
 *    for (int ii = 0; ii < out.size(); ii++)
 *              ROS_INFO_STREAM( " " << out[ii]);
 *
 */
    EXPECT_EQ ( true , flag );
  } 
 
  TEST_F ( EdgeOrientationExtractorTest, HorizontalEdge)
  {
    // The output vector
    std::vector<double>  out;
    EdgeOrientationExtractor::findLocalEdgeFeatures(horizontal, &out);
    ASSERT_EQ ( 5, out.size() ); 
    
    double normalizedEdges = (WIDTH - 2.0) / (HEIGHT * WIDTH);
    
    //uncomment to visualize
    /*
     *cv::imshow("horizontal", horizontal);
     *cv::waitKey(0);
     *
     *ROS_INFO_STREAM("EdgeFeatures= ");
     *for (int ii = 0; ii < out.size(); ii++)
     *          ROS_INFO_STREAM( " " << out[ii]);
     */

    EXPECT_EQ ( normalizedEdges , out[0] );
    EXPECT_EQ ( 0, out[1] );
    EXPECT_NEAR ( 0 , out[2], 0.00001 );
    EXPECT_NEAR ( 0 , out[3], 0.000010 );
    EXPECT_EQ ( 0 , out[4] );
  }
  
  TEST_F ( EdgeOrientationExtractorTest, VerticalEdge)
  {
    // The output vector
    std::vector<double>  out;
    EdgeOrientationExtractor::findLocalEdgeFeatures(vertical, &out);
    ASSERT_EQ ( 5, out.size() ); 
    
    double normalizedEdges = (HEIGHT - 2.0) / (HEIGHT * WIDTH);
    
    //uncomment to visualize
    /*
     *cv::imshow("vertical",vertical);
     *cv::waitKey(0);
     *
     *ROS_INFO_STREAM("EdgeFeatures= ");
     *for (int ii = 0; ii < out.size(); ii++)
     *          ROS_INFO_STREAM( " " << out[ii]);
     */

    EXPECT_EQ ( 0 , out[0] );
    EXPECT_EQ ( normalizedEdges, out[1] );
    EXPECT_NEAR ( 0 , out[2], 0.00001 );
    EXPECT_NEAR ( 0 , out[3], 0.000010 );
    EXPECT_EQ ( 0 , out[4] );

  }
 
  TEST_F ( EdgeOrientationExtractorTest, Diagonal45Edge)
  {
    // The output vector
    std::vector<double>  out;
    EdgeOrientationExtractor::findLocalEdgeFeatures(diagonal45, &out);
    ASSERT_EQ ( 5, out.size() ); 
    
    double normalizedEdges = (HEIGHT - 2.0) / (HEIGHT * WIDTH);

    //uncomment to visualize
/*
 *    cv::imshow("diagonal45",diagonal45);
 *    cv::waitKey(0);
 *    
 *    ROS_INFO_STREAM("EdgeFeatures= ");
 *    for (int ii = 0; ii < out.size(); ii++)
 *              ROS_INFO_STREAM( " " << out[ii]);
 *
*/
    EXPECT_EQ ( 0 , out[0] );
    EXPECT_EQ ( 0, out[1] );
    EXPECT_EQ ( 0 , out[2] );
    EXPECT_NE ( 0 , out[3] );
    EXPECT_EQ ( 0 , out[4] );

  }

  TEST_F ( EdgeOrientationExtractorTest, Diagonal135Edge)
  {
    // The output vector
    std::vector<double>  out;
    EdgeOrientationExtractor::findLocalEdgeFeatures(diagonal135, &out);
    ASSERT_EQ ( 5, out.size() ); 
    
    double normalizedEdges = (HEIGHT - 2.0) / (HEIGHT * WIDTH);
    //uncomment to visualize
    /*
     *cv::imshow("diagonal135",diagonal135);
     *cv::waitKey(0);
     *
     *ROS_INFO_STREAM("EdgeFeatures= ");
     *for (int ii = 0; ii < out.size(); ii++)
     *          ROS_INFO_STREAM( " " << out[ii]);
     */

    EXPECT_EQ ( 0 , out[0] );
    EXPECT_EQ ( 0, out[1] );
    EXPECT_NE ( 0 , out[2] );
    EXPECT_EQ ( 0 , out[3] );
    EXPECT_EQ ( 0 , out[4] );

  }

  /*TEST_F ( EdgeOrientationExtractorTest, CurvedEdges)
  {
    // The output vector
    std::vector<double>  out;
    EdgeOrientationExtractor::findLocalEdgeFeatures(circle, &out);
    ASSERT_EQ ( 5, out.size() ); 
    
    double normalizedEdges = (HEIGHT - 2.0) / (HEIGHT * WIDTH);
    //uncomment to vizualize
    [>
     *cv::imshow("circle",circle);
     *cv::waitKey(0);
     *
     *ROS_INFO_STREAM("EdgeFeatures= ");
     *for (int ii = 0; ii < out.size(); ii++)
     *          ROS_INFO_STREAM( " " << out[ii]);
     <]

    EXPECT_EQ ( 0 , out[0] );
    EXPECT_EQ ( 0, out[1] );
    EXPECT_NE ( 0 , out[2] );
    EXPECT_EQ ( 0 , out[3] );
    EXPECT_EQ ( 0 , out[4] );

  }*/
 
/*
 *  TEST_F ( EdgeOrientationExtractorTest, NoisyEdges)
 *  {
 *    // The output vector
 *    std::vector<double>  out;
 *    EdgeOrientationExtractor::findLocalEdgeFeatures(noisy, &out);
 *    ASSERT_EQ ( 5, out.size() ); 
 *    
 *    double normalizedEdges = (HEIGHT - 2.0) / (HEIGHT * WIDTH);
 *    //uncomment to vizualize
 *    cv::imshow("noisy",noisy);
 *    cv::waitKey(0);
 *    
 *    ROS_INFO_STREAM("EdgeFeatures= ");
 *    for (int ii = 0; ii < out.size(); ii++)
 *              ROS_INFO_STREAM( " " << out[ii]);
 *
 *    EXPECT_EQ ( 0 , out[0] );
 *    EXPECT_EQ ( 0, out[1] );
 *    EXPECT_NE ( 0 , out[2] );
 *    EXPECT_EQ ( 0 , out[3] );
 *    EXPECT_EQ ( 0 , out[4] );
 *
 *  }
 */

} // namespace pandora_vision

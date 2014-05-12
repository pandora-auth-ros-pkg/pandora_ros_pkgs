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
 * Author: Alexandros Philotheou
 *********************************************************************/

#include "utils/holes_conveyor.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class HolesConveyorUtilsTest
    @brief Tests the integrity of methods of class HolesConveyorUtils
   **/
  class HolesConveyorUtilsTest : public ::testing::Test
  {
    protected:

      HolesConveyorUtilsTest () {}

      virtual void SetUp()
      {
        // Dummy entries
        src.keyPoints.push_back ( cv::KeyPoint( 100, 100, 1 ) );
        src.keyPoints.push_back ( cv::KeyPoint( 200, 200, 1 ) );


        std::vector< cv::Point2f > src_rectangles;
        src_rectangles.push_back ( cv::Point2f ( 100, 100 ) );
        src.rectangles.push_back ( src_rectangles );

        src_rectangles.clear();
        src_rectangles.push_back ( cv::Point2f ( 200, 200 ) );
        src.rectangles.push_back ( src_rectangles );



        std::vector< cv::Point2f > src_outlines;
        src_outlines.push_back ( cv::Point2f ( 100, 100 ) );
        src.outlines.push_back ( src_outlines );

        src_outlines.clear();
        src_outlines.push_back ( cv::Point2f ( 200, 200 ) );
        src.outlines.push_back ( src_outlines );



        // Dummy entries
        dst.keyPoints.push_back ( cv::KeyPoint( 1, 1, 1 ) );
        dst.keyPoints.push_back ( cv::KeyPoint( 2, 2, 1 ) );


        std::vector< cv::Point2f > dst_rectangles;
        dst_rectangles.push_back ( cv::Point2f ( 1, 1 ) );
        dst.rectangles.push_back ( dst_rectangles );

        dst_rectangles.clear();
        dst_rectangles.push_back ( cv::Point2f ( 2, 2 ) );
        dst.rectangles.push_back ( dst_rectangles );



        std::vector< cv::Point2f > dst_outlines;
        dst_outlines.push_back ( cv::Point2f ( 1, 1 ) );
        dst.outlines.push_back ( dst_outlines );

        dst_outlines.clear();
        dst_outlines.push_back ( cv::Point2f ( 2, 2 ) );
        dst.outlines.push_back ( dst_outlines );

      }

      // This is the source conveyor
      HolesConveyor src;

      // And this is the destination conveyor
      HolesConveyor dst;
  };



  //! Test HolesConveyorUtils::append
  TEST_F ( HolesConveyorUtilsTest, AppendTest )
  {
    // 2 entries in dst before appending src to it
    ASSERT_EQ ( 2, HolesConveyorUtils::size(dst) );

    // Run HolesConveyorUtils::append
    HolesConveyorUtils::append( src, &dst );

    // 4 entries in dst after appending src to it
    EXPECT_EQ ( 4, HolesConveyorUtils::size(dst) );

    // Check that the initial entries have not been tampered with
    EXPECT_EQ ( 1, dst.rectangles[0][0].x );
    EXPECT_EQ ( 1, dst.outlines[0][0].x );
    EXPECT_EQ ( 1, dst.keyPoints[0].pt.x );

    EXPECT_EQ ( 1, dst.rectangles[0][0].y );
    EXPECT_EQ ( 1, dst.outlines[0][0].y );
    EXPECT_EQ ( 1, dst.keyPoints[0].pt.y );

    EXPECT_EQ ( 2, dst.rectangles[1][0].x );
    EXPECT_EQ ( 2, dst.outlines[1][0].x );
    EXPECT_EQ ( 2, dst.keyPoints[1].pt.x );

    EXPECT_EQ ( 2, dst.rectangles[1][0].y );
    EXPECT_EQ ( 2, dst.outlines[1][0].y );
    EXPECT_EQ ( 2, dst.keyPoints[1].pt.y );

    // Check the newly appended entries' elements against the original ones
    for ( int k = 0; k < src.keyPoints.size(); k++ )
    {
      EXPECT_NEAR ( dst.keyPoints[2 + k].pt.x, src.keyPoints[k].pt.x, 1 );
      EXPECT_NEAR ( dst.keyPoints[2 + k].pt.y, src.keyPoints[k].pt.y, 1 );

      for ( int r = 0; r < dst.rectangles[2 + k].size(); r++ )
      {
        EXPECT_NEAR ( dst.rectangles[2 + k][r].x, src.rectangles[k][r].x, 1 );
        EXPECT_NEAR ( dst.rectangles[2 + k][r].y, src.rectangles[k][r].y, 1 );
      }

      for ( int o = 0; o < dst.outlines[2 + k].size(); o++ )
      {
        EXPECT_NEAR ( dst.outlines[2 + k][o].x, src.outlines[k][o].x, 1 );
        EXPECT_NEAR ( dst.outlines[2 + k][o].y, src.outlines[k][o].y, 1 );
      }
    }
  }



  //! Test HolesConveyorUtils::appendDummyConveyor
  TEST_F ( HolesConveyorUtilsTest, AppendDummyConveyorTest )
  {
    // Run HolesConveyorUtils::appendDummyConveyor
    HolesConveyorUtils::appendDummyConveyor
      ( cv::Point2f ( 10, 10 ), cv::Point2f ( 20, 20 ), 10, 10, 5, 5, &dst);

    // There should now be three entries in dst
    ASSERT_EQ ( 3, HolesConveyorUtils::size( dst ) );

    // Check that the initial entries have not been tampered with
    EXPECT_EQ ( 1, dst.rectangles[0][0].x );
    EXPECT_EQ ( 1, dst.outlines[0][0].x );
    EXPECT_EQ ( 1, dst.keyPoints[0].pt.x );

    EXPECT_EQ ( 1, dst.rectangles[0][0].y );
    EXPECT_EQ ( 1, dst.outlines[0][0].y );
    EXPECT_EQ ( 1, dst.keyPoints[0].pt.y );

    EXPECT_EQ ( 2, dst.rectangles[1][0].x );
    EXPECT_EQ ( 2, dst.outlines[1][0].x );
    EXPECT_EQ ( 2, dst.keyPoints[1].pt.x );

    EXPECT_EQ ( 2, dst.rectangles[1][0].y );
    EXPECT_EQ ( 2, dst.outlines[1][0].y );
    EXPECT_EQ ( 2, dst.keyPoints[1].pt.y );

    // The new entry
    EXPECT_NEAR ( 22.5, dst.keyPoints[2].pt.x, 1 );
    EXPECT_NEAR ( 10, dst.rectangles[2][0].x, 1 );
    EXPECT_NEAR ( 20, dst.outlines[2][0].x, 1 );
  }



  //! Test HolesConveyorUtils::clear
  TEST_F ( HolesConveyorUtilsTest, ClearTest )
  {
    // Run HolesConveyorUtils::clear
    HolesConveyorUtils::clear( &dst );

    // The dst conveyor should be empty
    ASSERT_EQ ( 0, dst.keyPoints.size() );
    ASSERT_EQ ( 0, dst.rectangles.size() );
    ASSERT_EQ ( 0, dst.outlines.size() );
  }



  //! Test HolesConveyorUtils::copyTo
  TEST_F ( HolesConveyorUtilsTest, CopyToTest)
  {
    // Run HolesConveyorUtils::copyTo
    HolesConveyorUtils::copyTo( src, &dst );

    ASSERT_EQ ( 2, dst.keyPoints.size() );

    // Check the newly appended entries' elements against the original ones
    for ( int k = 0; k < src.keyPoints.size(); k++ )
    {
      EXPECT_NEAR ( dst.keyPoints[k].pt.x, src.keyPoints[k].pt.x, 1 );
      EXPECT_NEAR ( dst.keyPoints[k].pt.y, src.keyPoints[k].pt.y, 1 );

      for ( int r = 0; r < dst.rectangles[k].size(); r++ )
      {
        EXPECT_NEAR ( dst.rectangles[k][r].x, src.rectangles[k][r].x, 1 );
        EXPECT_NEAR ( dst.rectangles[k][r].y, src.rectangles[k][r].y, 1 );
      }

      for ( int o = 0; o < dst.outlines[k].size(); o++ )
      {
        EXPECT_NEAR ( dst.outlines[k][o].x, src.outlines[k][o].x, 1 );
        EXPECT_NEAR ( dst.outlines[k][o].y, src.outlines[k][o].y, 1 );
      }
    }
  }

} // namespace pandora_vision

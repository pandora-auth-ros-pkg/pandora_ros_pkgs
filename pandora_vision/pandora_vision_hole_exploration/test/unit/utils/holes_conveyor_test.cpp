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
        // Dummy entries for the src conveyor
        std::vector<cv::Point2f> keypoints;
        std::vector<cv::Rect> rectangles;

        cv::Point2f k_a ( 100, 100 );
        keypoints.push_back(k_a);
        cv::Rect r_a(100, 100, 100, 100);
        rectangles.push_back(r_a);


        cv::Point2f k_b ( 200, 200 );
        keypoints.push_back(k_b);
        cv::Rect r_b(200, 200, 200, 200);
        rectangles.push_back(r_b);

        // Push back the two holes into the src conveyor
        src.keypoint = keypoints;
        src.rectangle = rectangles;


        //// Dummy entries for the dst conveyor

        //HoleConveyor holeC;

        //cv::KeyPoint k_c ( 1, 1, 1 );

        //holeC.keypoint = k_c;
        //holeC.rectangle.push_back ( cv::Point2f ( 1, 1 ) );
        //holeC.outline.push_back ( cv::Point2f ( 1, 1 ) );

        //HoleConveyor holeD;

        //cv::KeyPoint k_d ( 2, 2, 1 );
        //holeD.keypoint = k_d;
        //holeD.rectangle.push_back ( cv::Point2f ( 2, 2 ) );
        //holeD.outline.push_back ( cv::Point2f ( 2, 2 ) );

        //// Push back the two holes into the dst conveyor
        //dst.holes.push_back(holeC);
        //dst.holes.push_back(holeD);

      }

      // This is the source conveyor
      HolesConveyor src;

      // This is the destination conveyor
      //HolesConveyor dst;
  };



  //! Tests HolesConveyorUtils::append
  //TEST_F ( HolesConveyorUtilsTest, appendTest )
  //{
  //  // Backup the original dst
  //  HolesConveyor dstBackup;
  //  HolesConveyorUtils::copyTo( dst, &dstBackup );

  //  // 2 entries in dst before appending src to it
  //  ASSERT_EQ ( 2, dst.size() );

  //  // Run HolesConveyorUtils::append
  //  HolesConveyorUtils::append( src, &dst );

  //  // 4 entries in dst after appending src to it
  //  EXPECT_EQ ( 4, dst.size() );

  //  // Check that the initial entries have not been tampered with
  //  for ( int i = 0; i < dstBackup.size(); i++ )
  //  {
  //    EXPECT_EQ ( dst.holes[i].keypoint.pt.x,
  //      dstBackup.holes[i].keypoint.pt.x );

  //    for ( int v = 0; v < dst.holes[i].rectangle.size(); v++ )
  //    {
  //      EXPECT_EQ ( dst.holes[i].rectangle[v].x,
  //        dstBackup.holes[i].rectangle[v].x );
  //      EXPECT_EQ ( dst.holes[i].rectangle[v].y,
  //        dstBackup.holes[i].rectangle[v].y );
  //    }

  //    for ( int o = 0; o < dst.holes[i].outline.size(); o++ )
  //    {
  //      EXPECT_EQ ( dst.holes[i].outline[o].x,
  //        dstBackup.holes[i].outline[o].x );
  //      EXPECT_EQ ( dst.holes[i].outline[o].y,
  //        dstBackup.holes[i].outline[o].y );
  //    }
  //  }

  //  // Check the newly appended entries' elements against the original ones
  //  for ( int k = 0; k < src.size(); k++ )
  //  {
  //    EXPECT_EQ ( dst.holes[2 + k].keypoint.pt.x, src.holes[k].keypoint.pt.x);
  //    EXPECT_EQ ( dst.holes[2 + k].keypoint.pt.y, src.holes[k].keypoint.pt.y);

  //    for ( int v = 0; v < dst.holes[k + 2].rectangle.size(); v++ )
  //    {
  //      EXPECT_EQ ( dst.holes[2 + k].rectangle[v].x,
  //        src.holes[k].rectangle[v].x );
  //      EXPECT_EQ ( dst.holes[2 + k].rectangle[v].y,
  //        src.holes[k].rectangle[v].y );
  //    }

  //    for ( int o = 0; o < dst.holes[k + 2].rectangle.size(); o++ )
  //    {
  //      EXPECT_EQ ( dst.holes[2 + k].rectangle[o].x,
  //        src.holes[k].rectangle[o].x );
  //      EXPECT_EQ ( dst.holes[2 + k].rectangle[o].y,
  //        src.holes[k].rectangle[o].y );
  //    }
  //  }
  //}



  //! Tests HolesConveyorUtils::appendDummyConveyor
  //TEST_F ( HolesConveyorUtilsTest, appendDummyConveyorTest )
  //{
  //  // Backup the original dst
  //  HolesConveyor dstBackup;
  //  HolesConveyorUtils::copyTo( dst, &dstBackup );

  //  // Run HolesConveyorUtils::appendDummyConveyor
  //  HolesConveyorUtils::appendDummyConveyor
  //    ( cv::Point2f ( 10, 10 ), cv::Point2f ( 20, 20 ), 10, 10, 5, 5, &dst);

  //  // There should now be three entries in dst
  //  ASSERT_EQ ( 3, dst.size() );

  //  // Check that the initial entries have not been tampered with
  //  for ( int i = 0; i < dstBackup.size(); i++ )
  //  {
  //    EXPECT_EQ ( dst.holes[i].keypoint.pt.x,
  //      dstBackup.holes[i].keypoint.pt.x );

  //    for ( int v = 0; v < dst.holes[i].rectangle.size(); v++ )
  //    {
  //      EXPECT_EQ ( dst.holes[i].rectangle[v].x,
  //        dstBackup.holes[i].rectangle[v].x );
  //      EXPECT_EQ ( dst.holes[i].rectangle[v].y,
  //        dstBackup.holes[i].rectangle[v].y );
  //    }

  //    for ( int o = 0; o < dst.holes[i].outline.size(); o++ )
  //    {
  //      EXPECT_EQ ( dst.holes[i].outline[o].x,
  //        dstBackup.holes[i].outline[o].x );
  //      EXPECT_EQ ( dst.holes[i].outline[o].y,
  //        dstBackup.holes[i].outline[o].y );
  //    }
  //  }

  //  // The new entry
  //  EXPECT_NEAR ( 22.5, dst.holes[2].keypoint.pt.x, 1 );
  //  EXPECT_NEAR ( 10, dst.holes[2].rectangle[0].x, 1 );
  //  EXPECT_NEAR ( 20, dst.holes[2].outline[0].x, 1 );
  //}



  //! Tests HolesConveyorUtils::clear
  TEST_F ( HolesConveyorUtilsTest, clearTest )
  {
    // Run HolesConveyorUtils::clear
    HolesConveyorUtils::clear( &src );

    // The conveyor should be empty
    ASSERT_EQ ( 0, src.size() );
  }



  //! Tests HolesConveyorUtils::copyTo
  //TEST_F ( HolesConveyorUtilsTest, copyToTest)
  //{
  //  // Run HolesConveyorUtils::copyTo
  //  HolesConveyorUtils::copyTo( src, &dst );

  //  // There should be two hole entries in dst now
  //  ASSERT_EQ ( 2, dst.size() );

  //  // Check the newly appended entries' elements against the original ones
  //  for ( int k = 0; k < src.size(); k++ )
  //  {
  //    EXPECT_NEAR ( dst.holes[k].keypoint.pt.x, src.holes[k].keypoint.pt.x, 1 );
  //    EXPECT_NEAR ( dst.holes[k].keypoint.pt.y, src.holes[k].keypoint.pt.y, 1 );

  //    for ( int r = 0; r < dst.holes[k].rectangle.size(); r++ )
  //    {
  //      EXPECT_NEAR ( dst.holes[k].rectangle[r].x,
  //        src.holes[k].rectangle[r].x, 1 );
  //      EXPECT_NEAR ( dst.holes[k].rectangle[r].y,
  //        src.holes[k].rectangle[r].y, 1 );
  //    }

  //    for ( int o = 0; o < dst.holes[k].outline.size(); o++ )
  //    {
  //      EXPECT_NEAR ( dst.holes[k].outline[o].x, src.holes[k].outline[o].x, 1 );
  //      EXPECT_NEAR ( dst.holes[k].outline[o].y, src.holes[k].outline[o].y, 1 );
  //    }
  //  }
  //}



  ////! Tests HolesConveyorUtils::generateRectangle
  //TEST_F ( HolesConveyorUtilsTest, generateRectangleTest )
  //{
  //  // The rectangle's points that will be returned
  //  std::vector< cv::Point2f > points_1;

  //  // The rectangle's vertices that will be returned
  //  std::vector< cv::Point2f > points_2;

  //  // Run HolesConveyorUtils::generateRectangle with intent = 1
  //  points_1 = HolesConveyorUtils::generateRectangle
  //    ( cv::Point2f ( 100, 100 ), 100, 100, 1 );

  //  // Run HolesConveyorUtils::generateRectangle with intent = 2
  //  points_2 = HolesConveyorUtils::generateRectangle
  //    ( cv::Point2f ( 100, 100 ), 100, 100, 2 );

  //  // The number of vertices should amount to a number
  //  // lower than the number of points
  //  ASSERT_LT ( points_2.size(), points_1.size() );

  //  // There should be four vertices
  //  EXPECT_EQ ( 4, points_2.size() );

  //  EXPECT_EQ ( 100, points_2[0].x );
  //  EXPECT_EQ ( 100, points_2[0].y );

  //  // There should be 4 * 100 points
  //  EXPECT_EQ ( 400, points_1.size() );

  //  EXPECT_EQ ( 100, points_1[0].x );
  //  EXPECT_EQ ( 100, points_1[0].y );

  //}



  //! Tests HolesConveyorUtils::getHole
  //TEST_F ( HolesConveyorUtilsTest, getHoleTest )
  //{
  //  // Run HolesConveyorUtils::getHole
  //  HolesConveyor hole = HolesConveyorUtils::getHole ( src, 0 );

  //  // There should be one entry inside hole
  //  ASSERT_EQ ( 1, hole.size() );

  //  // src and hole should have exactly the same amount of
  //  // rectangle and outline points
  //  ASSERT_EQ ( src.holes[0].rectangle.size(), hole.holes[0].rectangle.size() );
  //  ASSERT_EQ ( src.holes[0].outline.size(), hole.holes[0].outline.size() );

  //  // Check that the entries in src and hole are exactly the same
  //  EXPECT_EQ ( hole.holes[0].keypoint.pt.x, src.holes[0].keypoint.pt.x );
  //  EXPECT_EQ ( hole.holes[0].keypoint.pt.y, src.holes[0].keypoint.pt.y );

  //  for (int r = 0; r < hole.holes[0].rectangle.size(); r++)
  //  {
  //    EXPECT_EQ ( src.holes[0].rectangle[r].x, hole.holes[0].rectangle[r].x );
  //    EXPECT_EQ ( src.holes[0].rectangle[r].y, hole.holes[0].rectangle[r].y );
  //  }

  //  for (int o = 0; o < hole.holes[0].outline.size(); o++)
  //  {
  //    EXPECT_EQ ( src.holes[0].outline[o].x, hole.holes[0].outline[o].x );
  //    EXPECT_EQ ( src.holes[0].outline[o].y, hole.holes[0].outline[o].y );
  //  }
  //}



  //! Tests HolesConveyorUtils::merge
  //TEST_F ( HolesConveyorUtilsTest, mergeTest)
  //{
  //  // The merged conveyor
  //  HolesConveyor merged;

  //  // Merge src and dst into merged
  //  HolesConveyorUtils::merge( src, dst, &merged );

  //  // There should be exactly 2 + 2 = 4 entries in merged
  //  ASSERT_EQ ( 4, merged.size() );

  //  // Check the newly appended entries' elements against the original ones
  //  // Check src
  //  for ( int k = 0; k < src.size(); k++ )
  //  {
  //    EXPECT_NEAR ( merged.holes[k].keypoint.pt.x,
  //      src.holes[k].keypoint.pt.x, 1 );
  //    EXPECT_NEAR ( merged.holes[k].keypoint.pt.y,
  //      src.holes[k].keypoint.pt.y, 1 );

  //    for ( int r = 0; r < merged.holes[k].rectangle.size(); r++ )
  //    {
  //      EXPECT_NEAR ( merged.holes[k].rectangle[r].x,
  //        src.holes[k].rectangle[r].x, 1 );
  //      EXPECT_NEAR ( merged.holes[k].rectangle[r].y,
  //        src.holes[k].rectangle[r].y, 1 );
  //    }

  //    for ( int o = 0; o < merged.holes[k].outline.size(); o++ )
  //    {
  //      EXPECT_NEAR ( merged.holes[k].outline[o].x,
  //        src.holes[k].outline[o].x, 1 );
  //      EXPECT_NEAR ( merged.holes[k].outline[o].y,
  //        src.holes[k].outline[o].y, 1 );
  //    }
  //  }


  //  // Check dst
  //  for ( int k = 0; k < dst.size(); k++ )
  //  {
  //    EXPECT_NEAR ( merged.holes[k + 2].keypoint.pt.x,
  //      dst.holes[k].keypoint.pt.x, 1 );
  //    EXPECT_NEAR ( merged.holes[k + 2].keypoint.pt.y,
  //      dst.holes[k].keypoint.pt.y, 1 );

  //    for ( int r = 0; r < merged.holes[k].rectangle.size(); r++ )
  //    {
  //      EXPECT_NEAR ( merged.holes[k + 2].rectangle[r].x,
  //        dst.holes[k].rectangle[r].x, 1 );
  //      EXPECT_NEAR ( merged.holes[k + 2].rectangle[r].y,
  //        dst.holes[k].rectangle[r].y, 1 );
  //    }

  //    for ( int o = 0; o < merged.holes[k].outline.size(); o++ )
  //    {
  //      EXPECT_NEAR ( merged.holes[k + 2].outline[o].x,
  //        dst.holes[k].outline[o].x, 1 );
  //      EXPECT_NEAR ( merged.holes[k + 2].outline[o].y,
  //        dst.holes[k].outline[o].y, 1 );
  //    }
  //  }
  //}



  //! Tests HolesConveyorUtils::removeHole
  //TEST_F ( HolesConveyorUtilsTest, removeHoleTest )
  //{
  //  // Backup dst
  //  HolesConveyor dstBackup;
  //  HolesConveyorUtils::copyTo ( dst, &dstBackup );

  //  // Two holes before removing one
  //  ASSERT_EQ ( 2, dstBackup.size() );
  //  ASSERT_EQ ( 2, dst.size() );

  //  // Run HolesConveyorUtils::removeHole
  //  HolesConveyorUtils::removeHole ( &dst, 0 );

  //  // One hole after removing one
  //  ASSERT_EQ ( 1, dst.size() );

  //  // The first entry should be exactly the same as the previously second one
  //  EXPECT_NEAR ( dst.holes[0].keypoint.pt.x,
  //    dstBackup.holes[1].keypoint.pt.x, 1 );
  //  EXPECT_NEAR ( dst.holes[0].keypoint.pt.y,
  //    dstBackup.holes[1].keypoint.pt.y, 1 );

  //  for ( int r = 0; r < dst.holes[1].rectangle.size(); r++ )
  //  {
  //    EXPECT_NEAR ( dst.holes[0].rectangle[r].x,
  //      dstBackup.holes[1].rectangle[r].x, 1 );
  //    EXPECT_NEAR ( dst.holes[0].rectangle[r].y,
  //      dstBackup.holes[1].rectangle[r].y, 1 );
  //  }

  //  for ( int o = 0; o < dst.holes[1].outline.size(); o++ )
  //  {
  //    EXPECT_NEAR ( dst.holes[0].outline[o].x,
  //      dstBackup.holes[1].outline[o].x, 1 );
  //    EXPECT_NEAR ( dst.holes[0].outline[o].y,
  //      dstBackup.holes[1].outline[o].y, 1 );
  //  }
  //}



  //! Tests HolesConveyorUtils::replace
  //TEST_F ( HolesConveyorUtilsTest, replaceTest )
  //{
  //  // Run HolesConveyorUtils::replace
  //  HolesConveyorUtils::replace ( src, &dst );

  //  ASSERT_EQ ( 2, dst.size() );

  //  // Check the newly replaced entries' elements against the original ones
  //  for ( int k = 0; k < src.size(); k++ )
  //  {
  //    EXPECT_EQ ( dst.holes[k].keypoint.pt.x, src.holes[k].keypoint.pt.x );
  //    EXPECT_EQ ( dst.holes[k].keypoint.pt.y, src.holes[k].keypoint.pt.y );

  //    for ( int r = 0; r < dst.holes[k].rectangle.size(); r++ )
  //    {
  //      EXPECT_EQ ( dst.holes[k].rectangle[r].x, src.holes[k].rectangle[r].x );
  //      EXPECT_EQ ( dst.holes[k].rectangle[r].y, src.holes[k].rectangle[r].y );
  //    }

  //    for ( int o = 0; o < dst.holes[k].outline.size(); o++ )
  //    {
  //      EXPECT_EQ ( dst.holes[k].outline[o].x, src.holes[k].outline[o].x );
  //      EXPECT_EQ ( dst.holes[k].outline[o].y, src.holes[k].outline[o].y );
  //    }
  //  }
  //}



  //! Tests HolesConveyorUtils::replaceHole
  //TEST_F ( HolesConveyorUtilsTest, replaceHoleTest )
  //{
  //  // Backup dst
  //  HolesConveyor dstBackup;
  //  HolesConveyorUtils::copyTo( dst, &dstBackup );

  //  // Run HolesConveyorUtils::replaceHole
  //  HolesConveyorUtils::replaceHole ( src, 0, &dst, 0 );

  //  // There should still be two holes inside dst
  //  ASSERT_EQ ( 2, dst.size() );

  //  // The 0-th hole of dst should be the 0-th hole of src
  //  EXPECT_EQ ( dst.holes[0].keypoint.pt.x, src.holes[0].keypoint.pt.x );
  //  EXPECT_EQ ( dst.holes[0].keypoint.pt.y, src.holes[0].keypoint.pt.y );

  //  for ( int r = 0; r < dst.holes[0].rectangle.size(); r++ )
  //  {
  //    EXPECT_EQ ( dst.holes[0].rectangle[r].x, src.holes[0].rectangle[r].x );
  //    EXPECT_EQ ( dst.holes[0].rectangle[r].y, src.holes[0].rectangle[r].y );
  //  }

  //  for ( int o = 0; o < dst.holes[0].outline.size(); o++ )
  //  {
  //    EXPECT_EQ ( dst.holes[0].outline[o].x, src.holes[0].outline[o].x );
  //    EXPECT_EQ ( dst.holes[0].outline[o].y, src.holes[0].outline[o].y );
  //  }

  //  // The 1-st hole of dst should be the 1-st hole of dstBackup
  //  EXPECT_EQ ( dst.holes[1].keypoint.pt.x, dstBackup.holes[1].keypoint.pt.x );
  //  EXPECT_EQ ( dst.holes[1].keypoint.pt.y, dstBackup.holes[1].keypoint.pt.y );

  //  for ( int r = 0; r < dst.holes[0].rectangle.size(); r++ )
  //  {
  //    EXPECT_EQ ( dst.holes[1].rectangle[r].x,
  //      dstBackup.holes[1].rectangle[r].x );
  //    EXPECT_EQ ( dst.holes[1].rectangle[r].y,
  //      dstBackup.holes[1].rectangle[r].y );
  //  }

  //  for ( int o = 0; o < dst.holes[0].outline.size(); o++ )
  //  {
  //    EXPECT_EQ ( dst.holes[1].outline[o].x, dstBackup.holes[1].outline[o].x );
  //    EXPECT_EQ ( dst.holes[1].outline[o].y, dstBackup.holes[1].outline[o].y );
  //  }
  //}



  //! Tests HolesConveyorUtils::shuffle
  //TEST_F ( HolesConveyorUtilsTest, shuffleTest )
  //{
  //  // Backup dst
  //  HolesConveyor dstBackup;
  //  HolesConveyorUtils::copyTo ( dst, &dstBackup );

  //  // Run HolesConveyorUtils::shuffle
  //  HolesConveyorUtils::shuffle ( &dst );

  //  // There should still be 2 entries in dst
  //  ASSERT_EQ ( 2, dst.size() );

  //  if ( dst.holes[0].keypoint.pt.x == dstBackup.holes[0].keypoint.pt.x )
  //  {
  //    EXPECT_EQ ( dst.holes[0].keypoint.pt.x, dstBackup.holes[0].keypoint.pt.x );
  //    EXPECT_EQ ( dst.holes[0].keypoint.pt.y, dstBackup.holes[0].keypoint.pt.y );

  //    for ( int r = 0; r < dst.holes[0].rectangle.size(); r++ )
  //    {
  //      EXPECT_EQ ( dst.holes[0].rectangle[r].x,
  //        dstBackup.holes[0].rectangle[r].x );
  //      EXPECT_EQ ( dst.holes[0].rectangle[r].y,
  //        dstBackup.holes[0].rectangle[r].y );
  //    }

  //    for ( int o = 0; o < dst.holes[0].outline.size(); o++ )
  //    {
  //      EXPECT_EQ ( dst.holes[0].outline[o].x, dstBackup.holes[0].outline[o].x );
  //      EXPECT_EQ ( dst.holes[0].outline[o].y, dstBackup.holes[0].outline[o].y );
  //    }
  //  }
  //  else if ( dst.holes[0].keypoint.pt.x == dstBackup.holes[1].keypoint.pt.x )
  //  {
  //    EXPECT_EQ ( dst.holes[0].keypoint.pt.x, dstBackup.holes[1].keypoint.pt.x );
  //    EXPECT_EQ ( dst.holes[0].keypoint.pt.y, dstBackup.holes[1].keypoint.pt.y );

  //    for ( int r = 0; r < dst.holes[0].rectangle.size(); r++ )
  //    {
  //      EXPECT_EQ ( dst.holes[0].rectangle[r].x,
  //        dstBackup.holes[1].rectangle[r].x );
  //      EXPECT_EQ ( dst.holes[0].rectangle[r].y,
  //        dstBackup.holes[1].rectangle[r].y );
  //    }

  //    for ( int o = 0; o < dst.holes[0].outline.size(); o++ )
  //    {
  //      EXPECT_EQ ( dst.holes[0].outline[o].x, dstBackup.holes[1].outline[o].x );
  //      EXPECT_EQ ( dst.holes[0].outline[o].y, dstBackup.holes[1].outline[o].y );
  //    }
  //  }
  //}

} // namespace pandora_vision

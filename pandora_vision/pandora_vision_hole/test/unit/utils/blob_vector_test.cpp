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

#include "utils/message_conversions.h"
#include "utils/blob_vector.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class BlobVectorTest
    @brief Tests the integrity of methods of class BlobVector
   **/
  class BlobVectorTest : public ::testing::Test
  {
    protected:

      BlobVectorTest () {}

      virtual void SetUp()
      {
        // Dummy entries for the src conveyor

        pandora_vision_msgs::Blob holeA;

        holeA.areaOfInterest.center.x = 100;
        holeA.areaOfInterest.center.y = 100;
        holeA.areaOfInterest.width = 5;
        holeA.areaOfInterest.height = 5;
        holeA.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(100, 100)));

        pandora_vision_msgs::Blob holeB;

        holeB.areaOfInterest.center.x = 200;
        holeB.areaOfInterest.center.y = 200;
        holeB.areaOfInterest.width = 10;
        holeB.areaOfInterest.height = 10;
        holeB.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(200, 200)));

        // Push back the two holes into the src conveyor
        src.append(holeA);
        src.append(holeB);

        // Dummy entries for the dst conveyor

        pandora_vision_msgs::Blob holeC;

        holeC.areaOfInterest.center.x = 1;
        holeC.areaOfInterest.center.y = 1;
        holeC.areaOfInterest.width = 1;
        holeC.areaOfInterest.height = 1;
        holeC.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(1, 1)));

        pandora_vision_msgs::Blob holeD;

        holeD.areaOfInterest.center.x = 2;
        holeD.areaOfInterest.center.y = 2;
        holeD.areaOfInterest.width = 2;
        holeD.areaOfInterest.height = 2;
        holeD.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(2, 2)));

        // Push back the two holes into the dst conveyor
        dst.append(holeC);
        dst.append(holeD);
      }
      // This is the source conveyor
      BlobVector src;

      // This is the destination conveyor
      BlobVector dst;
  };

  //! Tests BlobVector::extend
  TEST_F ( BlobVectorTest, extendTest )
  {
    // Backup the original dst
    BlobVector dstBackup;
    dstBackup.copy(dst);

    // 2 entries in dst before appending src to it
    ASSERT_EQ ( 2, dst.size() );

    // Run BlobVector::extend
    dst.extend(src);

    // 4 entries in dst after appending src to it
    EXPECT_EQ ( 4, dst.size() );

    // Check that the initial entries have not been tampered with
    for ( int i = 0; i < dstBackup.size(); i++ )
    {
      EXPECT_EQ ( dst.getBlob(i).areaOfInterest.center.x,
        dstBackup.getBlob(i).areaOfInterest.center.x );

      EXPECT_EQ ( dst.getBlob(i).areaOfInterest.width,
        dstBackup.getBlob(i).areaOfInterest.width );

      for ( int o = 0; o < dst.getBlob(i).outline.size(); o++ )
      {
        EXPECT_EQ ( dst.getBlob(i).outline[o].x,
          dstBackup.getBlob(i).outline[o].x );
        EXPECT_EQ ( dst.getBlob(i).outline[o].y,
          dstBackup.getBlob(i).outline[o].y );
      }
    }

    // Check the newly appended entries' elements against the original ones
    for ( int k = 0; k < src.size(); k++ )
    {
      EXPECT_EQ (dst.getBlob(2 + k).areaOfInterest.center.x,
        src.getBlob(k).areaOfInterest.center.x);
      EXPECT_EQ (dst.getBlob(2 + k).areaOfInterest.center.y,
        src.getBlob(k).areaOfInterest.center.y);

      EXPECT_EQ (dst.getBlob(2 + k).areaOfInterest.width,
        src.getBlob(k).areaOfInterest.width);
      EXPECT_EQ (dst.getBlob(2 + k).areaOfInterest.height,
        src.getBlob(k).areaOfInterest.height);
    }
  }

  //! Tests HolesConveyorUtils::appendDummyConveyor
  TEST_F ( BlobVectorTest, appendDummyConveyorTest )
  {
    // Backup the original dst
    BlobVector dstBackup;
    dstBackup.copy(dst);

    // Run BlobVector::append
    dst.append
      (cv::Point2f ( 10, 10 ), cv::Point2f ( 20, 20 ), 10, 10, 5, 5);

    // There should now be three entries in dst
    ASSERT_EQ ( 3, dst.size() );

    // Check that the initial entries have not been tampered with
    for ( int i = 0; i < dstBackup.size(); i++ )
    {
      EXPECT_EQ ( dst.getBlob(i).areaOfInterest.center.x,
        dstBackup.getBlob(i).areaOfInterest.center.x );

      EXPECT_EQ ( dst.getBlob(i).areaOfInterest.width,
        dstBackup.getBlob(i).areaOfInterest.width );

      for ( int o = 0; o < dst.getBlob(i).outline.size(); o++ )
      {
        EXPECT_EQ ( dst.getBlob(i).outline[o].x,
          dstBackup.getBlob(i).outline[o].x );
        EXPECT_EQ ( dst.getBlob(i).outline[o].y,
          dstBackup.getBlob(i).outline[o].y );
      }
    }
    // The new entry
    EXPECT_NEAR ( 15, dst.getBlob(2).areaOfInterest.center.x, 1 );
    EXPECT_NEAR ( 10, dst.getBlob(2).areaOfInterest.width, 1 );  //
    EXPECT_NEAR ( 20, dst.getBlob(2).outline[0].x, 1 );
  }

  //! Tests BlobVector::clear
  TEST_F ( BlobVectorTest, clearTest )
  {
    // Run BlobVector::clear
    dst.clear();

    // The dst conveyor should be empty
    ASSERT_EQ ( 0, dst.size() );
  }

  //! Tests BlobVector::copy
  TEST_F ( BlobVectorTest, copyTest)
  {
    // Run BlobVector::copy
    dst.copy(src);

    // There should be two hole entries in dst now
    ASSERT_EQ ( 2, dst.size() );

    // Check the newly appended entries' elements against the original ones
    for ( int k = 0; k < src.size(); k++ )
    {
      EXPECT_NEAR ( dst.getBlob(k).areaOfInterest.center.x, 
        src.getBlob(k).areaOfInterest.center.x, 1 );
      EXPECT_NEAR ( dst.getBlob(k).areaOfInterest.center.y, 
        src.getBlob(k).areaOfInterest.center.y, 1 );

      EXPECT_NEAR ( dst.getBlob(k).areaOfInterest.width, 
        src.getBlob(k).areaOfInterest.width, 1 );
      EXPECT_NEAR ( dst.getBlob(k).areaOfInterest.height, 
        src.getBlob(k).areaOfInterest.height, 1 );

      for ( int o = 0; o < dst.getBlob(k).outline.size(); o++ )
      {
        EXPECT_NEAR ( dst.getBlob(k).outline[o].x, src.getBlob(k).outline[o].x, 1 );
        EXPECT_NEAR ( dst.getBlob(k).outline[o].y, src.getBlob(k).outline[o].y, 1 );
      }
    }
  }

  //! Tests HolesConveyorUtils::generateRectangle
  //~ TEST_F ( HolesConveyorUtilsTest, generateRectangleTest )
  //~ {
    //~ // The rectangle's points that will be returned
    //~ std::vector< cv::Point2f > points_1;
//~ 
    //~ // The rectangle's vertices that will be returned
    //~ std::vector< cv::Point2f > points_2;
//~ 
    //~ // Run HolesConveyorUtils::generateRectangle with intent = 1
    //~ points_1 = HolesConveyorUtils::generateRectangle
      //~ ( cv::Point2f ( 100, 100 ), 100, 100, 1 );
//~ 
    //~ // Run HolesConveyorUtils::generateRectangle with intent = 2
    //~ points_2 = HolesConveyorUtils::generateRectangle
      //~ ( cv::Point2f ( 100, 100 ), 100, 100, 2 );
//~ 
    //~ // The number of vertices should amount to a number
    //~ // lower than the number of points
    //~ ASSERT_LT ( points_2.size(), points_1.size() );
//~ 
    //~ // There should be four vertices
    //~ EXPECT_EQ ( 4, points_2.size() );
//~ 
    //~ EXPECT_EQ ( 100, points_2[0].x );
    //~ EXPECT_EQ ( 100, points_2[0].y );
//~ 
    //~ // There should be 4 * 100 points
    //~ EXPECT_EQ ( 400, points_1.size() );
//~ 
    //~ EXPECT_EQ ( 100, points_1[0].x );
    //~ EXPECT_EQ ( 100, points_1[0].y );
//~ 
  //~ }

  //! Tests BlobVector::getBlob
  TEST_F ( BlobVectorTest, getBlobTest )
  {
    // Run BlobVector::getBlob
    BlobVector hole;
    hole.append(src.getBlob(0));

    // There should be one entry inside hole
    ASSERT_EQ ( 1, hole.size() );

    // src and hole should have exactly the same amount of
    // outline points
    ASSERT_EQ ( src.getBlob(0).outline.size(), hole.getBlob(0).outline.size() );

    // Check that the entries in src and hole are exactly the same
    EXPECT_EQ ( hole.getBlob(0).areaOfInterest.center.x, src.getBlob(0).areaOfInterest.center.x );
    EXPECT_EQ ( hole.getBlob(0).areaOfInterest.center.y, src.getBlob(0).areaOfInterest.center.y );

    EXPECT_EQ ( hole.getBlob(0).areaOfInterest.width, src.getBlob(0).areaOfInterest.width );
    EXPECT_EQ ( hole.getBlob(0).areaOfInterest.height, src.getBlob(0).areaOfInterest.height );

    for (int o = 0; o < hole.getBlob(0).outline.size(); o++)
    {
      EXPECT_EQ ( src.getBlob(0).outline[o].x, hole.getBlob(0).outline[o].x );
      EXPECT_EQ ( src.getBlob(0).outline[o].y, hole.getBlob(0).outline[o].y );
    }
  }

  //! Tests BlobVector::merge
  TEST_F ( BlobVectorTest, mergeTest)
  {
    // The merged conveyor
    BlobVector merged;

    // Merge src and dst into merged
    merged.merge(src, dst);

    // There should be exactly 2 + 2 = 4 entries in merged
    ASSERT_EQ ( 4, merged.size() );

    // Check the newly appended entries' elements against the original ones
    // Check src
    for ( int k = 0; k < src.size(); k++ )
    {
      EXPECT_NEAR ( merged.getBlob(k).areaOfInterest.center.x,
        src.getBlob(k).areaOfInterest.center.x, 1 );
      EXPECT_NEAR ( merged.getBlob(k).areaOfInterest.center.y,
        src.getBlob(k).areaOfInterest.center.y, 1 );

      EXPECT_NEAR ( merged.getBlob(k).areaOfInterest.width,
        src.getBlob(k).areaOfInterest.width, 1 );
      EXPECT_NEAR ( merged.getBlob(k).areaOfInterest.height,
        src.getBlob(k).areaOfInterest.height, 1 );

      for ( int o = 0; o < merged.getBlob(k).outline.size(); o++ )
      {
        EXPECT_NEAR ( merged.getBlob(k).outline[o].x,
          src.getBlob(k).outline[o].x, 1 );
        EXPECT_NEAR ( merged.getBlob(k).outline[o].y,
          src.getBlob(k).outline[o].y, 1 );
      }
    }

    // Check dst
    for ( int k = 0; k < dst.size(); k++ )
    {
      EXPECT_NEAR ( merged.getBlob(k + 2).areaOfInterest.center.x,
        dst.getBlob(k).areaOfInterest.center.x, 1 );
      EXPECT_NEAR ( merged.getBlob(k + 2).areaOfInterest.center.y,
        dst.getBlob(k).areaOfInterest.center.y, 1 );

      EXPECT_NEAR ( merged.getBlob(k + 2).areaOfInterest.width,
        dst.getBlob(k).areaOfInterest.width, 1 );
      EXPECT_NEAR ( merged.getBlob(k + 2).areaOfInterest.height,
        dst.getBlob(k).areaOfInterest.height, 1 );

      for ( int o = 0; o < merged.getBlob(k).outline.size(); o++ )
      {
        EXPECT_NEAR ( merged.getBlob(k + 2).outline[o].x,
          dst.getBlob(k).outline[o].x, 1 );
        EXPECT_NEAR ( merged.getBlob(k + 2).outline[o].y,
          dst.getBlob(k).outline[o].y, 1 );
      }
    }
  }

  //! Tests BlobVector::removeHole
  TEST_F ( BlobVectorTest, removeHoleTest )
  {
    // Backup dst
    BlobVector dstBackup;
    dstBackup.copy(dst);

    // Two holes before removing one
    ASSERT_EQ ( 2, dstBackup.size() );
    ASSERT_EQ ( 2, dst.size() );

    // Run BlobVector::removeHole
    dst.removeHole(0);

    // One hole after removing one
    ASSERT_EQ ( 1, dst.size() );

    // The first entry should be exactly the same as the previously second one
    EXPECT_NEAR ( dst.getBlob(0).areaOfInterest.center.x,
      dstBackup.getBlob(1).areaOfInterest.center.x, 1 );
    EXPECT_NEAR ( dst.getBlob(0).areaOfInterest.center.y,
      dstBackup.getBlob(1).areaOfInterest.center.y, 1 );

    EXPECT_NEAR ( dst.getBlob(0).areaOfInterest.width,
      dstBackup.getBlob(1).areaOfInterest.width, 1 );
    EXPECT_NEAR ( dst.getBlob(0).areaOfInterest.height,
      dstBackup.getBlob(1).areaOfInterest.height, 1 );

    for ( int o = 0; o < dst.getBlob(1).outline.size(); o++ )
    {
      EXPECT_NEAR ( dst.getBlob(0).outline[o].x,
        dstBackup.getBlob(1).outline[o].x, 1 );
      EXPECT_NEAR ( dst.getBlob(0).outline[o].y,
        dstBackup.getBlob(1).outline[o].y, 1 );
    }
  }

  //! Tests BlobVector::replace
  TEST_F ( BlobVectorTest, replaceTest )
  {
    // Run BlobVector::replace
    dst.replace(src);

    ASSERT_EQ ( 2, dst.size() );

    // Check the newly replaced entries' elements against the original ones
    for ( int k = 0; k < src.size(); k++ )
    {
      EXPECT_EQ ( dst.getBlob(k).areaOfInterest.center.x, src.getBlob(k).areaOfInterest.center.x );
      EXPECT_EQ ( dst.getBlob(k).areaOfInterest.center.y, src.getBlob(k).areaOfInterest.center.y );

      EXPECT_EQ ( dst.getBlob(k).areaOfInterest.width, src.getBlob(k).areaOfInterest.width );
      EXPECT_EQ ( dst.getBlob(k).areaOfInterest.height, src.getBlob(k).areaOfInterest.height );

      for ( int o = 0; o < dst.getBlob(k).outline.size(); o++ )
      {
        EXPECT_EQ ( dst.getBlob(k).outline[o].x, src.getBlob(k).outline[o].x );
        EXPECT_EQ ( dst.getBlob(k).outline[o].y, src.getBlob(k).outline[o].y );
      }
    }
  }

  //! Tests BlobVector::replaceHole
  TEST_F ( BlobVectorTest, replaceHoleTest )
  {
    // Backup dst
    BlobVector dstBackup;
    dstBackup.copy(dst);

    // Run BlobVector::replaceHole
    dst.replaceHole(src, 0, 0);

    // There should still be two holes inside dst
    ASSERT_EQ ( 2, dst.size() );

    // The 0-th hole of dst should be the 0-th hole of src
    EXPECT_EQ ( dst.getBlob(0).areaOfInterest.center.x, src.getBlob(0).areaOfInterest.center.x );
    EXPECT_EQ ( dst.getBlob(0).areaOfInterest.center.y, src.getBlob(0).areaOfInterest.center.y );

    EXPECT_EQ ( dst.getBlob(0).areaOfInterest.width, src.getBlob(0).areaOfInterest.width );
    EXPECT_EQ ( dst.getBlob(0).areaOfInterest.height, src.getBlob(0).areaOfInterest.height );

    for ( int o = 0; o < dst.getBlob(0).outline.size(); o++ )
    {
      EXPECT_EQ ( dst.getBlob(0).outline[o].x, src.getBlob(0).outline[o].x );
      EXPECT_EQ ( dst.getBlob(0).outline[o].y, src.getBlob(0).outline[o].y );
    }

    // The 1-st hole of dst should be the 1-st hole of dstBackup
    EXPECT_EQ ( dst.getBlob(1).areaOfInterest.center.x, dstBackup.getBlob(1).areaOfInterest.center.x );
    EXPECT_EQ ( dst.getBlob(1).areaOfInterest.center.y, dstBackup.getBlob(1).areaOfInterest.center.y );

    EXPECT_EQ ( dst.getBlob(1).areaOfInterest.width, dstBackup.getBlob(1).areaOfInterest.width );
    EXPECT_EQ ( dst.getBlob(1).areaOfInterest.height, dstBackup.getBlob(1).areaOfInterest.height );

    for ( int o = 0; o < dst.getBlob(0).outline.size(); o++ )
    {
      EXPECT_EQ ( dst.getBlob(1).outline[o].x, dstBackup.getBlob(1).outline[o].x );
      EXPECT_EQ ( dst.getBlob(1).outline[o].y, dstBackup.getBlob(1).outline[o].y );
    }
  }

  //! Tests BlobVector::shuffle
  TEST_F ( BlobVectorTest, shuffleTest )
  {
    // Backup dst
    BlobVector dstBackup;
    dstBackup.copy(dst);

    // Run BlobVector::shuffle
    dst.shuffle();

    // There should still be 2 entries in dst
    ASSERT_EQ ( 2, dst.size() );

    if ( dst.getBlob(0).areaOfInterest.center.x == dstBackup.getBlob(0).areaOfInterest.center.x )
    {
      EXPECT_EQ ( dst.getBlob(0).areaOfInterest.center.x,
        dstBackup.getBlob(0).areaOfInterest.center.x );
      EXPECT_EQ ( dst.getBlob(0).areaOfInterest.center.y,
        dstBackup.getBlob(0).areaOfInterest.center.y );

      EXPECT_EQ ( dst.getBlob(0).areaOfInterest.width,
        dstBackup.getBlob(0).areaOfInterest.width );
      EXPECT_EQ ( dst.getBlob(0).areaOfInterest.height,
        dstBackup.getBlob(0).areaOfInterest.height );

      for ( int o = 0; o < dst.getBlob(0).outline.size(); o++ )
      {
        EXPECT_EQ ( dst.getBlob(0).outline[o].x, dstBackup.getBlob(0).outline[o].x );
        EXPECT_EQ ( dst.getBlob(0).outline[o].y, dstBackup.getBlob(0).outline[o].y );
      }
    }
    else if ( dst.getBlob(0).areaOfInterest.center.x == dstBackup.getBlob(1).areaOfInterest.center.x )
    {
      EXPECT_EQ ( dst.getBlob(0).areaOfInterest.center.x,
        dstBackup.getBlob(1).areaOfInterest.center.x );
      EXPECT_EQ ( dst.getBlob(0).areaOfInterest.center.y,
        dstBackup.getBlob(1).areaOfInterest.center.y );

      EXPECT_EQ ( dst.getBlob(0).areaOfInterest.width,
        dstBackup.getBlob(1).areaOfInterest.width );
      EXPECT_EQ ( dst.getBlob(0).areaOfInterest.height,
        dstBackup.getBlob(1).areaOfInterest.height );

      for ( int o = 0; o < dst.getBlob(0).outline.size(); o++ )
      {
        EXPECT_EQ ( dst.getBlob(0).outline[o].x, dstBackup.getBlob(1).outline[o].x );
        EXPECT_EQ ( dst.getBlob(0).outline[o].y, dstBackup.getBlob(1).outline[o].y );
      }
    }
  }

}  // namespace pandora_vision

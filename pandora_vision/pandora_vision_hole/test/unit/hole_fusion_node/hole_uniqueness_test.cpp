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

#include "hole_fusion_node/hole_uniqueness.h"
#include "utils/visualization.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  /**
    @class HoleUniquenessTest
    @brief Tests the integrity of methods of class HoleUniqueness
   **/
  class HoleUniquenessTest : public ::testing::Test
  {
    protected:

      HoleUniquenessTest(){}

      virtual void SetUp(){}
  };



  //! Tests HoleUniqueness::makeHolesUnique (a)
  TEST_F ( HoleUniquenessTest, makeHolesUniqueA )
  {
    // The container of holes
    HolesConveyor conveyor;

    // A container of one hole
    HoleConveyor hole_1;

    // Construct hole_1
    cv::KeyPoint k_1 ( 100.0, 100.0, 1 );

    hole_1.keypoint = k_1;

    hole_1.rectangle.push_back( cv::Point2f( 80.0 , 80.0 ) );
    hole_1.rectangle.push_back( cv::Point2f( 80.0 , 120.0 ) );
    hole_1.rectangle.push_back( cv::Point2f( 120.0 , 120.0 ) );
    hole_1.rectangle.push_back( cv::Point2f( 120.0 , 80.0 ) );

    hole_1.outline.push_back( cv::Point2f( 80.0 , 80.0 ) );
    hole_1.outline.push_back( cv::Point2f( 80.0 , 120.0 ) );
    hole_1.outline.push_back( cv::Point2f( 120.0 , 80.0 ) );
    hole_1.outline.push_back( cv::Point2f( 120.0 , 120.0 ) );


    // A container of another hole
    HoleConveyor hole_2;

    // Construct hole_2
    cv::KeyPoint k_2 ( 90.0, 90.0, 1 );

    hole_2.keypoint = k_2;

    hole_2.rectangle.push_back( cv::Point2f( 70.0 , 70.0 ) );
    hole_2.rectangle.push_back( cv::Point2f( 70.0 , 130.0 ) );
    hole_2.rectangle.push_back( cv::Point2f( 130.0 , 130.0 ) );
    hole_2.rectangle.push_back( cv::Point2f( 130.0 , 70.0 ) );

    hole_2.outline.push_back( cv::Point2f( 70.0 , 70.0 ) );
    hole_2.outline.push_back( cv::Point2f( 70.0 , 130.0 ) );
    hole_2.outline.push_back( cv::Point2f( 130.0 , 70.0 ) );
    hole_2.outline.push_back( cv::Point2f( 130.0 , 130.0 ) );


    // A container of a third hole
    HoleConveyor hole_3;

    // Construct hole_3
    cv::KeyPoint k_3 ( 80.0, 80.0, 1 );

    hole_3.keypoint = k_3;

    hole_3.rectangle.push_back( cv::Point2f( 60.0 , 60.0 ) );
    hole_3.rectangle.push_back( cv::Point2f( 60.0 , 120.0 ) );
    hole_3.rectangle.push_back( cv::Point2f( 120.0 , 120.0 ) );
    hole_3.rectangle.push_back( cv::Point2f( 120.0 , 60.0 ) );

    hole_3.outline.push_back( cv::Point2f( 60.0 , 60.0 ) );
    hole_3.outline.push_back( cv::Point2f( 60.0 , 120.0 ) );
    hole_3.outline.push_back( cv::Point2f( 120.0 , 60.0 ) );
    hole_3.outline.push_back( cv::Point2f( 120.0 , 120.0 ) );


    // Holes 1 and 2 will be multiple, in a random order.
    conveyor.holes.push_back( hole_1 );
    conveyor.holes.push_back( hole_2 );
    conveyor.holes.push_back( hole_3 );
    conveyor.holes.push_back( hole_2 );
    conveyor.holes.push_back( hole_2 );
    conveyor.holes.push_back( hole_1 );

    // Run HoleUniqueness::makeHolesUnique (a)
    HoleUniqueness::makeHolesUnique( &conveyor );


    // There should be only three unique holes inside the container
    EXPECT_EQ ( 3, conveyor.size() );

    EXPECT_EQ ( 80.0, conveyor.holes[0].keypoint.pt.x );
    EXPECT_EQ ( 80.0, conveyor.holes[0].keypoint.pt.y );

    EXPECT_EQ ( 100.0, conveyor.holes[1].keypoint.pt.x );
    EXPECT_EQ ( 100.0, conveyor.holes[1].keypoint.pt.y );

    EXPECT_EQ ( 90.0, conveyor.holes[2].keypoint.pt.x );
    EXPECT_EQ ( 90.0, conveyor.holes[2].keypoint.pt.y );

  }



  //! Tests HoleUniqueness::makeHolesUnique (b)
  TEST_F ( HoleUniquenessTest, makeHolesUniqueB )
  {
    // What one needs to do here, is to construct a swarm of holes
    // around some point and give each hole a validity probability.

    // First swarm; swarm a
    HoleConveyor hole_a1;
    HoleConveyor hole_a2;
    HoleConveyor hole_a3;

    cv::KeyPoint a ( 100.0, 100.0, 1 );

    hole_a1.keypoint = a;
    hole_a2.keypoint = a;
    hole_a3.keypoint = a;

    hole_a1.rectangle.push_back( cv::Point2f( 80.0 , 80.0 ) );
    hole_a1.rectangle.push_back( cv::Point2f( 80.0 , 120.0 ) );
    hole_a1.rectangle.push_back( cv::Point2f( 120.0 , 120.0 ) );
    hole_a1.rectangle.push_back( cv::Point2f( 120.0 , 80.0 ) );

    hole_a1.outline.push_back( cv::Point2f( 80.0 , 80.0 ) );
    hole_a1.outline.push_back( cv::Point2f( 120.0 , 80.0 ) );
    hole_a1.outline.push_back( cv::Point2f( 80.0 , 120.0 ) );
    hole_a1.outline.push_back( cv::Point2f( 120.0 , 120.0 ) );


    hole_a2.rectangle.push_back( cv::Point2f( 70.0 , 70.0 ) );
    hole_a2.rectangle.push_back( cv::Point2f( 70.0 , 120.0 ) );
    hole_a2.rectangle.push_back( cv::Point2f( 120.0 , 120.0 ) );
    hole_a2.rectangle.push_back( cv::Point2f( 120.0 , 70.0 ) );

    hole_a2.outline.push_back( cv::Point2f( 70.0 , 70.0 ) );
    hole_a2.outline.push_back( cv::Point2f( 120.0 , 70.0 ) );
    hole_a2.outline.push_back( cv::Point2f( 70.0 , 120.0 ) );
    hole_a2.outline.push_back( cv::Point2f( 120.0 , 120.0 ) );


    hole_a3.rectangle.push_back( cv::Point2f( 70.0 , 70.0 ) );
    hole_a3.rectangle.push_back( cv::Point2f( 70.0 , 130.0 ) );
    hole_a3.rectangle.push_back( cv::Point2f( 130.0 , 130.0 ) );
    hole_a3.rectangle.push_back( cv::Point2f( 130.0 , 70.0 ) );

    hole_a3.outline.push_back( cv::Point2f( 70.0 , 70.0 ) );
    hole_a3.outline.push_back( cv::Point2f( 130.0 , 70.0 ) );
    hole_a3.outline.push_back( cv::Point2f( 70.0 , 130.0 ) );
    hole_a3.outline.push_back( cv::Point2f( 130.0 , 130.0 ) );


    // Second swarm; swarm b
    HoleConveyor hole_b1;
    HoleConveyor hole_b2;

    cv::KeyPoint b ( 200.0, 200.0, 1 );

    hole_b1.keypoint = b;
    hole_b2.keypoint = b;

    hole_b1.rectangle.push_back( cv::Point2f( 180.0 , 180.0 ) );
    hole_b1.rectangle.push_back( cv::Point2f( 180.0 , 220.0 ) );
    hole_b1.rectangle.push_back( cv::Point2f( 220.0 , 220.0 ) );
    hole_b1.rectangle.push_back( cv::Point2f( 220.0 , 180.0 ) );

    hole_b1.outline.push_back( cv::Point2f( 180.0 , 180.0 ) );
    hole_b1.outline.push_back( cv::Point2f( 220.0 , 180.0 ) );
    hole_b1.outline.push_back( cv::Point2f( 180.0 , 220.0 ) );
    hole_b1.outline.push_back( cv::Point2f( 220.0 , 220.0 ) );


    hole_b2.rectangle.push_back( cv::Point2f( 150.0 , 150.0 ) );
    hole_b2.rectangle.push_back( cv::Point2f( 280.0 , 150.0 ) );
    hole_b2.rectangle.push_back( cv::Point2f( 280.0 , 280.0 ) );
    hole_b2.rectangle.push_back( cv::Point2f( 150.0 , 280.0 ) );

    hole_b2.outline.push_back( cv::Point2f( 150.0 , 150.0 ) );
    hole_b2.outline.push_back( cv::Point2f( 150.0 , 280.0 ) );
    hole_b2.outline.push_back( cv::Point2f( 280.0 , 150.0 ) );
    hole_b2.outline.push_back( cv::Point2f( 280.0 , 280.0 ) );


    // Add another, unique, hole
    HoleConveyor hole;

    cv::KeyPoint c ( 300.0, 300.0, 1 );

    hole.keypoint = c;

    hole.rectangle.push_back( cv::Point2f( 280.0 , 280.0 ) );
    hole.rectangle.push_back( cv::Point2f( 280.0 , 320.0 ) );
    hole.rectangle.push_back( cv::Point2f( 320.0 , 320.0 ) );
    hole.rectangle.push_back( cv::Point2f( 320.0 , 280.0 ) );

    hole.outline.push_back( cv::Point2f( 280.0 , 280.0 ) );
    hole.outline.push_back( cv::Point2f( 320.0 , 280.0 ) );
    hole.outline.push_back( cv::Point2f( 280.0 , 320.0 ) );
    hole.outline.push_back( cv::Point2f( 320.0 , 320.0 ) );


    // Push all the holes back into a container
    HolesConveyor conveyor;
    conveyor.holes.push_back( hole_a1 );
    conveyor.holes.push_back( hole_a2 );
    conveyor.holes.push_back( hole_a3 );
    conveyor.holes.push_back( hole_b1 );
    conveyor.holes.push_back( hole_b2 );
    conveyor.holes.push_back( hole );


    // Construct a validity map
    std::map< int, float > validityMap;
    validityMap[0] = 0.9;
    validityMap[1] = 0.8;
    validityMap[2] = 0.91;
    validityMap[3] = 0.9;
    validityMap[4] = 0.8;
    validityMap[5] = 0.9;

    // Run HoleUniqueness::makeHolesUnique (b)
    HoleUniqueness::makeHolesUnique( &conveyor, &validityMap );

    // There should be three unique holes
    ASSERT_EQ( 3, conveyor.size() );

    std::map< int, float >::iterator it = validityMap.begin();

    EXPECT_NEAR( 0.91, it->second, 0.01 );

    it++;
    EXPECT_NEAR( 0.9, it->second, 0.01 );

    it++;
    EXPECT_NEAR( 0.9, it->second, 0.01 );


    // Inquire about the internals of the unique holes
    EXPECT_NEAR( 100.0, conveyor.holes[0].keypoint.pt.x, 0.01 );
    EXPECT_NEAR( 100.0, conveyor.holes[0].keypoint.pt.y, 0.01 );

    EXPECT_NEAR( 70.0, conveyor.holes[0].rectangle[0].x, 0.01 );
    EXPECT_NEAR( 70.0, conveyor.holes[0].rectangle[0].y, 0.01 );
    EXPECT_NEAR( 70.0, conveyor.holes[0].rectangle[1].x, 0.01 );
    EXPECT_NEAR( 130.0, conveyor.holes[0].rectangle[1].y, 0.01 );
    EXPECT_NEAR( 130.0, conveyor.holes[0].rectangle[2].x, 0.01 );
    EXPECT_NEAR( 130.0, conveyor.holes[0].rectangle[2].y, 0.01 );
    EXPECT_NEAR( 130.0, conveyor.holes[0].rectangle[3].x, 0.01 );
    EXPECT_NEAR( 70.0, conveyor.holes[0].rectangle[3].y, 0.01 );


    EXPECT_NEAR( 200.0, conveyor.holes[1].keypoint.pt.x, 0.01 );
    EXPECT_NEAR( 200.0, conveyor.holes[1].keypoint.pt.y, 0.01 );

    EXPECT_NEAR( 180.0, conveyor.holes[1].rectangle[0].x, 0.01 );
    EXPECT_NEAR( 180.0, conveyor.holes[1].rectangle[0].y, 0.01 );
    EXPECT_NEAR( 180.0, conveyor.holes[1].rectangle[1].x, 0.01 );
    EXPECT_NEAR( 220.0, conveyor.holes[1].rectangle[1].y, 0.01 );
    EXPECT_NEAR( 220.0, conveyor.holes[1].rectangle[2].x, 0.01 );
    EXPECT_NEAR( 220.0, conveyor.holes[1].rectangle[2].y, 0.01 );
    EXPECT_NEAR( 220.0, conveyor.holes[1].rectangle[3].x, 0.01 );
    EXPECT_NEAR( 180.0, conveyor.holes[1].rectangle[3].y, 0.01 );


    EXPECT_NEAR( 300.0, conveyor.holes[2].keypoint.pt.x, 0.01 );
    EXPECT_NEAR( 300.0, conveyor.holes[2].keypoint.pt.y, 0.01 );

    EXPECT_NEAR( 280.0, conveyor.holes[2].rectangle[0].x, 0.01 );
    EXPECT_NEAR( 280.0, conveyor.holes[2].rectangle[0].y, 0.01 );
    EXPECT_NEAR( 280.0, conveyor.holes[2].rectangle[1].x, 0.01 );
    EXPECT_NEAR( 320.0, conveyor.holes[2].rectangle[1].y, 0.01 );
    EXPECT_NEAR( 320.0, conveyor.holes[2].rectangle[2].x, 0.01 );
    EXPECT_NEAR( 320.0, conveyor.holes[2].rectangle[2].y, 0.01 );
    EXPECT_NEAR( 320.0, conveyor.holes[2].rectangle[3].x, 0.01 );
    EXPECT_NEAR( 280.0, conveyor.holes[2].rectangle[3].y, 0.01 );

  }

} // namespace pandora_vision

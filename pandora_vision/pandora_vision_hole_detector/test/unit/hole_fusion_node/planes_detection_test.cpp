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

#include "hole_fusion_node/planes_detection.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  /**
    @class PlanesDetectionTest
    @brief Tests the integrity of methods of class PlanesDetection
   **/
  class PlanesDetectionTest : public ::testing::Test
  {
    protected:

      PlanesDetectionTest() : cloud ( new PointCloudXYZ ) {}

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // Construct the point cloud

        cloud->height = HEIGHT;
        cloud->width = WIDTH;
        cloud->resize ( cloud->height * cloud->width );

        int i = 0;
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++ )
          {
            cloud->points[i].x = 1.0 + rows / 10;
            cloud->points[i].y = 1.0 + cols / 10;

            if (cols < WIDTH / 4)
            {
              cloud->points[i].z = 1.0;
            }
            else
            {
              cloud->points[i].z = 20.0;
            }

            i++;
          }
        }
      }

      // The height and width of the point cloud
      int HEIGHT;
      int WIDTH;

      // The point cloud which the test are relied on
      PointCloudXYZPtr cloud;
  };



  //! Tests PlanesDetection::applyVoxelGridFilter
  TEST_F ( PlanesDetectionTest, applyVoxelGridFilterTest )
  {
    // Run PlanesDetection::applyVoxelGridFilter
    PointCloudXYZPtr cloudFiltered =  PlanesDetection::applyVoxelGridFilter( cloud );

    EXPECT_LT ( cloudFiltered->points.size(), cloud->points.size() );
  }



  //! Tests PlanesDetection::locatePlanes
  TEST_F ( PlanesDetectionTest, locatePlanesTest )
  {
    std::vector<pcl::PointIndices::Ptr> inliersVector;

    // Run PlanesDetection::locatePlanes without applying voxel filtering
    int numPlanes = PlanesDetection::locatePlanes( cloud, false, &inliersVector );

    // There should be two planes detected
    EXPECT_EQ ( 2, numPlanes );

    // The first plane is comprised of the three quarters
    // of the entire point cloud
    EXPECT_EQ ( WIDTH * HEIGHT * 3 / 4, inliersVector[0]->indices.size() );

    // The remaining quarter is the second point cloud
    EXPECT_EQ ( WIDTH * HEIGHT / 4, inliersVector[1]->indices.size() );

    // Run PlanesDetection::locatePlanes with voxel filtering
    inliersVector.clear();

    numPlanes = PlanesDetection::locatePlanes( cloud, true, &inliersVector );

    // There should be two planes detected
    EXPECT_EQ ( 2, numPlanes );

    // The first plane is comprised of the one hundredth of three quarters
    // of the entire point cloud
    EXPECT_EQ ( WIDTH * HEIGHT * 3 / 4 / 100, inliersVector[0]->indices.size() );

    // The remaining one hundredth quarter is the second point cloud
    EXPECT_EQ ( WIDTH * HEIGHT / 4 / 100, inliersVector[1]->indices.size() );
  }



  //! Tests PlanesDetection::locatePlanesUsingSACSegmentation
  TEST_F ( PlanesDetectionTest, locatePlanesUsingSACSegmentationTest )
  {
    // The planar point clouds' vector
    std::vector<PointCloudXYZPtr> planesVector;

    // The vector of coefficients per plane
    std::vector<pcl::ModelCoefficients> coefficientsVector;

    // The vector of inliers per plane
    std::vector<pcl::PointIndices::Ptr> inliersVector;

    PlanesDetection::locatePlanesUsingSACSegmentation(
      cloud,
      &planesVector,
      &coefficientsVector,
      &inliersVector );


    // There should be two planes detected
    EXPECT_EQ ( 2, planesVector.size() );

    // The first plane is comprised of the three quarters
    // of the entire point cloud
    EXPECT_EQ ( WIDTH * HEIGHT * 3 / 4, inliersVector[0]->indices.size() );

    // The remaining quarter is the second point cloud
    EXPECT_EQ ( WIDTH * HEIGHT / 4, inliersVector[1]->indices.size() );
  }

} // namespace pandora_vision

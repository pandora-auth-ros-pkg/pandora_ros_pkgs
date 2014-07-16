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

#include "hole_fusion_node/hole_validation.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  /**
    @class HoleValidationTest
    @brief Tests the integrity of methods of class HoleValidation
   **/
  class HoleValidationTest : public ::testing::Test
  {
    protected:

      HoleValidationTest(){}

      virtual void SetUp(){}

      // A vector of probabilities for when in RGBD_MODE,
      // when all available filters are active
      std::vector< std::vector< float > > probabilitiesVectorAllRGBDMode;

      // A vector of probabilities for when in RGBD_MODE,
      // when some of the available filters are active
      std::vector< std::vector< float > > probabilitiesVectorSomeRGBDMode;

      // A vector of probabilities for when in RGB_ONLY_MODE,
      // when all available filters are active
      std::vector< std::vector< float > > probabilitiesVectorAllRGBOnlyMode;

      // A vector of probabilities for when in RGB_ONLY_MODE,
      // when some of the available filters are active
      std::vector< std::vector< float > > probabilitiesVectorSomeRGBOnlyMode;
  };



  //! Tests HoleValidation::validateHoles
  TEST_F ( HoleValidationTest, validateHolesTest )
  {
    //////////////////////////////// RGBD_MODE /////////////////////////////////

    //////////////////////////////// All filters ///////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 1;
    Parameters::Filters::DepthArea::priority = 2;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 3;
    Parameters::Filters::RectanglePlaneConstitution::priority = 4;
    Parameters::Filters::DepthHomogeneity::priority = 5;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 6;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 7;
    Parameters::Filters::DepthDiff::priority = 8;
    Parameters::Filters::TextureDiff::rgbd_priority = 9;


    // Set up filters' responses about two holes

    // Colour homogeneity
    std::vector< float > colourHomogeneityValues;
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Depth / area
    std::vector< float > depthAreaValues;
    depthAreaValues.push_back( 0.0 );
    depthAreaValues.push_back( 1.0 );

    // Luminosity diff
    std::vector< float > luminosityDiffValues;
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Rectangle plane constitution
    std::vector< float > rectanglePlaneConstitutionValues;
    rectanglePlaneConstitutionValues.push_back( 0.0 );
    rectanglePlaneConstitutionValues.push_back( 1.0 );

    // Depth homogeneity
    std::vector< float > depthHomogeneityValues;
    depthHomogeneityValues.push_back( 0.0 );
    depthHomogeneityValues.push_back( 1.0 );

    // Texture backprojection
    std::vector< float > textureBackprojectionValues;
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Intermediate points plane constitution
    std::vector< float > intermediatePointsPlaneConstitutionValues;
    intermediatePointsPlaneConstitutionValues.push_back( 0.0 );
    intermediatePointsPlaneConstitutionValues.push_back( 1.0 );

    // Depth diff
    std::vector< float > depthDiffValues;
    depthDiffValues.push_back( 0.0 );
    depthDiffValues.push_back( 1.0 );

    // Texture diff
    std::vector< float > textureDiffValues;
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorAllRGBDMode.push_back( colourHomogeneityValues );
    probabilitiesVectorAllRGBDMode.push_back( depthAreaValues );
    probabilitiesVectorAllRGBDMode.push_back( luminosityDiffValues );
    probabilitiesVectorAllRGBDMode.push_back( rectanglePlaneConstitutionValues );
    probabilitiesVectorAllRGBDMode.push_back( depthHomogeneityValues );
    probabilitiesVectorAllRGBDMode.push_back( textureBackprojectionValues );
    probabilitiesVectorAllRGBDMode.push_back( intermediatePointsPlaneConstitutionValues );
    probabilitiesVectorAllRGBDMode.push_back( depthDiffValues );
    probabilitiesVectorAllRGBDMode.push_back( textureDiffValues );


    // Run HoleValidation::validateHoles for RGBD_MODE
    for (int vp = 0; vp < 3; vp++)
    {
      Parameters::HoleFusion::Validation::validation_process = vp;

      std::map< int, float > validityMap =
        HoleValidation::validateHoles(
          probabilitiesVectorAllRGBDMode, RGBD_MODE );

      // Only the second hole should have been deemed valid
      ASSERT_EQ ( 1, validityMap.size() );

      std::map< int, float>::iterator v_it = validityMap.begin();

      ASSERT_EQ ( 1, v_it->first);
      ASSERT_EQ ( 1.0, v_it->second);
    }


    /////////////////////////////// Some filters ///////////////////////////////

    // Set the priority of some available filters
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 1;
    Parameters::Filters::DepthArea::priority = 2;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 0;
    Parameters::Filters::RectanglePlaneConstitution::priority = 3;
    Parameters::Filters::DepthHomogeneity::priority = 0;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 4;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 5;
    Parameters::Filters::DepthDiff::priority = 6;
    Parameters::Filters::TextureDiff::rgbd_priority = 0;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Depth / area
    depthAreaValues.clear();
    depthAreaValues.push_back( 0.0 );
    depthAreaValues.push_back( 1.0 );

    // Rectangle plane constitution
    rectanglePlaneConstitutionValues.clear();
    rectanglePlaneConstitutionValues.push_back( 0.0 );
    rectanglePlaneConstitutionValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Intermediate points plane constitution
    intermediatePointsPlaneConstitutionValues.clear();
    intermediatePointsPlaneConstitutionValues.push_back( 0.0 );
    intermediatePointsPlaneConstitutionValues.push_back( 1.0 );

    // Texture diff
    textureDiffValues.clear();
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorSomeRGBDMode.push_back( colourHomogeneityValues );
    probabilitiesVectorSomeRGBDMode.push_back( depthAreaValues );
    probabilitiesVectorSomeRGBDMode.push_back( rectanglePlaneConstitutionValues );
    probabilitiesVectorSomeRGBDMode.push_back( textureBackprojectionValues );
    probabilitiesVectorSomeRGBDMode.push_back( intermediatePointsPlaneConstitutionValues );
    probabilitiesVectorSomeRGBDMode.push_back( textureDiffValues );


    // Run HoleValidation::validateHolesfor RGBD_MODE
    for (int vp = 0; vp < 3; vp++)
    {
      Parameters::HoleFusion::Validation::validation_process = vp;

      std::map< int, float > validityMap =
        HoleValidation::validateHoles(
          probabilitiesVectorSomeRGBDMode, RGBD_MODE );

      // Only the second hole should have been deemed valid
      ASSERT_EQ ( 1, validityMap.size() );

      std::map< int, float>::iterator v_it = validityMap.begin();

      ASSERT_EQ ( 1, v_it->first);
      ASSERT_EQ ( 1.0, v_it->second);
    }



    ////////////////////////////// RGB_ONLY_MODE ///////////////////////////////

    /////////////////////////////// All filters ////////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgb_priority = 1;
    Parameters::Filters::LuminosityDiff::rgb_priority = 2;
    Parameters::Filters::TextureBackprojection::rgb_priority = 3;
    Parameters::Filters::TextureDiff::rgb_priority = 4;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Luminosity diff
    luminosityDiffValues.clear();
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Texture diff
    textureDiffValues.clear();
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorAllRGBOnlyMode.push_back( colourHomogeneityValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( luminosityDiffValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( textureBackprojectionValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( textureDiffValues );

    // Run HoleValidation::validateHolesfor RGB_ONLY_MODE
    for (int vp = 0; vp < 3; vp++)
    {
      Parameters::HoleFusion::Validation::validation_process = vp;

      std::map< int, float > validityMap =
        HoleValidation::validateHoles(
          probabilitiesVectorAllRGBOnlyMode, RGB_ONLY_MODE );

      // Only the second hole should have been deemed valid
      ASSERT_EQ ( 1, validityMap.size() );

      std::map< int, float>::iterator v_it = validityMap.begin();

      ASSERT_EQ ( 1, v_it->first);
      ASSERT_EQ ( 1.0, v_it->second);
    }


    /////////////////////////////// Some filters ///////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgb_priority = 1;
    Parameters::Filters::LuminosityDiff::rgb_priority = 2;
    Parameters::Filters::TextureBackprojection::rgb_priority = 3;
    Parameters::Filters::TextureDiff::rgb_priority = 0;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Luminosity diff
    luminosityDiffValues.clear();
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorSomeRGBOnlyMode.push_back( colourHomogeneityValues );
    probabilitiesVectorSomeRGBOnlyMode.push_back( luminosityDiffValues );
    probabilitiesVectorSomeRGBOnlyMode.push_back( textureBackprojectionValues );

    // Run HoleValidation::validateHolesfor RGB_ONLY_MODE
    for (int vp = 0; vp < 3; vp++)
    {
      Parameters::HoleFusion::Validation::validation_process = vp;

      std::map< int, float > validityMap =
        HoleValidation::validateHoles(
          probabilitiesVectorSomeRGBOnlyMode, RGB_ONLY_MODE );

      // Only the second hole should have been deemed valid
      ASSERT_EQ ( 1, validityMap.size() );

      std::map< int, float>::iterator v_it = validityMap.begin();

      ASSERT_EQ ( 1, v_it->first);
      ASSERT_EQ ( 1.0, v_it->second);
    }
  }



  //! Tests HoleValidation::validateHolesViaThresholdedWeighting
  TEST_F ( HoleValidationTest, validateHolesViaThresholdedWeightingTest )
  {
    //////////////////////////////// RGBD_MODE /////////////////////////////////

    //////////////////////////////// All filters ///////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 1;
    Parameters::Filters::DepthArea::priority = 2;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 3;
    Parameters::Filters::RectanglePlaneConstitution::priority = 4;
    Parameters::Filters::DepthHomogeneity::priority = 5;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 6;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 7;
    Parameters::Filters::DepthDiff::priority = 8;
    Parameters::Filters::TextureDiff::rgbd_priority = 9;


    // Set up filters' responses about two holes

    // Colour homogeneity
    std::vector< float > colourHomogeneityValues;
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Depth / area
    std::vector< float > depthAreaValues;
    depthAreaValues.push_back( 0.0 );
    depthAreaValues.push_back( 1.0 );

    // Luminosity diff
    std::vector< float > luminosityDiffValues;
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Rectangle plane constitution
    std::vector< float > rectanglePlaneConstitutionValues;
    rectanglePlaneConstitutionValues.push_back( 0.0 );
    rectanglePlaneConstitutionValues.push_back( 1.0 );

    // Depth homogeneity
    std::vector< float > depthHomogeneityValues;
    depthHomogeneityValues.push_back( 0.0 );
    depthHomogeneityValues.push_back( 1.0 );

    // Texture backprojection
    std::vector< float > textureBackprojectionValues;
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Intermediate points plane constitution
    std::vector< float > intermediatePointsPlaneConstitutionValues;
    intermediatePointsPlaneConstitutionValues.push_back( 0.0 );
    intermediatePointsPlaneConstitutionValues.push_back( 1.0 );

    // Depth diff
    std::vector< float > depthDiffValues;
    depthDiffValues.push_back( 0.0 );
    depthDiffValues.push_back( 1.0 );

    // Texture diff
    std::vector< float > textureDiffValues;
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorAllRGBDMode.push_back( colourHomogeneityValues );
    probabilitiesVectorAllRGBDMode.push_back( depthAreaValues );
    probabilitiesVectorAllRGBDMode.push_back( luminosityDiffValues );
    probabilitiesVectorAllRGBDMode.push_back( rectanglePlaneConstitutionValues );
    probabilitiesVectorAllRGBDMode.push_back( depthHomogeneityValues );
    probabilitiesVectorAllRGBDMode.push_back( textureBackprojectionValues );
    probabilitiesVectorAllRGBDMode.push_back( intermediatePointsPlaneConstitutionValues );
    probabilitiesVectorAllRGBDMode.push_back( depthDiffValues );
    probabilitiesVectorAllRGBDMode.push_back( textureDiffValues );


    // Run HoleValidation::validateHolesViaThresholdedWeighting for RGBD_MODE
    std::map< int, float > validityMap =
      HoleValidation::validateHolesViaThresholdedWeighting(
        probabilitiesVectorAllRGBDMode, RGBD_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    std::map< int, float>::iterator v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);


    /////////////////////////////// Some filters ///////////////////////////////

    // Set the priority of some available filters
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 1;
    Parameters::Filters::DepthArea::priority = 2;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 0;
    Parameters::Filters::RectanglePlaneConstitution::priority = 3;
    Parameters::Filters::DepthHomogeneity::priority = 0;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 4;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 5;
    Parameters::Filters::DepthDiff::priority = 6;
    Parameters::Filters::TextureDiff::rgbd_priority = 0;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Depth / area
    depthAreaValues.clear();
    depthAreaValues.push_back( 0.0 );
    depthAreaValues.push_back( 1.0 );

    // Rectangle plane constitution
    rectanglePlaneConstitutionValues.clear();
    rectanglePlaneConstitutionValues.push_back( 0.0 );
    rectanglePlaneConstitutionValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Intermediate points plane constitution
    intermediatePointsPlaneConstitutionValues.clear();
    intermediatePointsPlaneConstitutionValues.push_back( 0.0 );
    intermediatePointsPlaneConstitutionValues.push_back( 1.0 );

    // Texture diff
    textureDiffValues.clear();
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorSomeRGBDMode.push_back( colourHomogeneityValues );
    probabilitiesVectorSomeRGBDMode.push_back( depthAreaValues );
    probabilitiesVectorSomeRGBDMode.push_back( rectanglePlaneConstitutionValues );
    probabilitiesVectorSomeRGBDMode.push_back( textureBackprojectionValues );
    probabilitiesVectorSomeRGBDMode.push_back( intermediatePointsPlaneConstitutionValues );
    probabilitiesVectorSomeRGBDMode.push_back( textureDiffValues );


    // Run HoleValidation::validateHolesViaThresholdedWeighting for RGBD_MODE
    validityMap =
      HoleValidation::validateHolesViaThresholdedWeighting(
        probabilitiesVectorSomeRGBDMode, RGBD_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);



    ////////////////////////////// RGB_ONLY_MODE ///////////////////////////////

    /////////////////////////////// All filters ////////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgb_priority = 1;
    Parameters::Filters::LuminosityDiff::rgb_priority = 2;
    Parameters::Filters::TextureBackprojection::rgb_priority = 3;
    Parameters::Filters::TextureDiff::rgb_priority = 4;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Luminosity diff
    luminosityDiffValues.clear();
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Texture diff
    textureDiffValues.clear();
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorAllRGBOnlyMode.push_back( colourHomogeneityValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( luminosityDiffValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( textureBackprojectionValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( textureDiffValues );

    // Run HoleValidation::validateHolesViaThresholdedWeighting for RGB_ONLY_MODE
    validityMap = HoleValidation::validateHolesViaThresholdedWeighting(
        probabilitiesVectorAllRGBOnlyMode, RGB_ONLY_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);


    /////////////////////////////// Some filters ///////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgb_priority = 1;
    Parameters::Filters::LuminosityDiff::rgb_priority = 2;
    Parameters::Filters::TextureBackprojection::rgb_priority = 3;
    Parameters::Filters::TextureDiff::rgb_priority = 0;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Luminosity diff
    luminosityDiffValues.clear();
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorSomeRGBOnlyMode.push_back( colourHomogeneityValues );
    probabilitiesVectorSomeRGBOnlyMode.push_back( luminosityDiffValues );
    probabilitiesVectorSomeRGBOnlyMode.push_back( textureBackprojectionValues );

    // Run HoleValidation::validateHolesViaThresholdedWeighting for RGB_ONLY_MODE
    validityMap = HoleValidation::validateHolesViaThresholdedWeighting(
        probabilitiesVectorSomeRGBOnlyMode, RGB_ONLY_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);
  }



  //! Tests HoleValidation::validateHolesViaThresholding
  TEST_F ( HoleValidationTest, validateHolesViaThresholdingTest )
  {
    //////////////////////////////// RGBD_MODE /////////////////////////////////

    //////////////////////////////// All filters ///////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 1;
    Parameters::Filters::DepthArea::priority = 2;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 3;
    Parameters::Filters::RectanglePlaneConstitution::priority = 4;
    Parameters::Filters::DepthHomogeneity::priority = 5;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 6;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 7;
    Parameters::Filters::DepthDiff::priority = 8;
    Parameters::Filters::TextureDiff::rgbd_priority = 9;


    // Set up filters' responses about two holes

    // Colour homogeneity
    std::vector< float > colourHomogeneityValues;
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Depth / area
    std::vector< float > depthAreaValues;
    depthAreaValues.push_back( 0.0 );
    depthAreaValues.push_back( 1.0 );

    // Luminosity diff
    std::vector< float > luminosityDiffValues;
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Rectangle plane constitution
    std::vector< float > rectanglePlaneConstitutionValues;
    rectanglePlaneConstitutionValues.push_back( 0.0 );
    rectanglePlaneConstitutionValues.push_back( 1.0 );

    // Depth homogeneity
    std::vector< float > depthHomogeneityValues;
    depthHomogeneityValues.push_back( 0.0 );
    depthHomogeneityValues.push_back( 1.0 );

    // Texture backprojection
    std::vector< float > textureBackprojectionValues;
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Intermediate points plane constitution
    std::vector< float > intermediatePointsPlaneConstitutionValues;
    intermediatePointsPlaneConstitutionValues.push_back( 0.0 );
    intermediatePointsPlaneConstitutionValues.push_back( 1.0 );

    // Depth diff
    std::vector< float > depthDiffValues;
    depthDiffValues.push_back( 0.0 );
    depthDiffValues.push_back( 1.0 );

    // Texture diff
    std::vector< float > textureDiffValues;
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorAllRGBDMode.push_back( colourHomogeneityValues );
    probabilitiesVectorAllRGBDMode.push_back( depthAreaValues );
    probabilitiesVectorAllRGBDMode.push_back( luminosityDiffValues );
    probabilitiesVectorAllRGBDMode.push_back( rectanglePlaneConstitutionValues );
    probabilitiesVectorAllRGBDMode.push_back( depthHomogeneityValues );
    probabilitiesVectorAllRGBDMode.push_back( textureBackprojectionValues );
    probabilitiesVectorAllRGBDMode.push_back( intermediatePointsPlaneConstitutionValues );
    probabilitiesVectorAllRGBDMode.push_back( depthDiffValues );
    probabilitiesVectorAllRGBDMode.push_back( textureDiffValues );


    // Run HoleValidation::validateHolesViaThresholding for RGBD_MODE
    std::map< int, float > validityMap =
      HoleValidation::validateHolesViaThresholding(
        probabilitiesVectorAllRGBDMode, RGBD_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    std::map< int, float>::iterator v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);


    /////////////////////////////// Some filters ///////////////////////////////

    // Set the priority of some available filters
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 1;
    Parameters::Filters::DepthArea::priority = 2;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 0;
    Parameters::Filters::RectanglePlaneConstitution::priority = 3;
    Parameters::Filters::DepthHomogeneity::priority = 0;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 4;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 5;
    Parameters::Filters::DepthDiff::priority = 6;
    Parameters::Filters::TextureDiff::rgbd_priority = 0;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Depth / area
    depthAreaValues.clear();
    depthAreaValues.push_back( 0.0 );
    depthAreaValues.push_back( 1.0 );

    // Rectangle plane constitution
    rectanglePlaneConstitutionValues.clear();
    rectanglePlaneConstitutionValues.push_back( 0.0 );
    rectanglePlaneConstitutionValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Intermediate points plane constitution
    intermediatePointsPlaneConstitutionValues.clear();
    intermediatePointsPlaneConstitutionValues.push_back( 0.0 );
    intermediatePointsPlaneConstitutionValues.push_back( 1.0 );

    // Texture diff
    textureDiffValues.clear();
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorSomeRGBDMode.push_back( colourHomogeneityValues );
    probabilitiesVectorSomeRGBDMode.push_back( depthAreaValues );
    probabilitiesVectorSomeRGBDMode.push_back( rectanglePlaneConstitutionValues );
    probabilitiesVectorSomeRGBDMode.push_back( textureBackprojectionValues );
    probabilitiesVectorSomeRGBDMode.push_back( intermediatePointsPlaneConstitutionValues );
    probabilitiesVectorSomeRGBDMode.push_back( textureDiffValues );


    // Run HoleValidation::validateHolesViaThresholding for RGBD_MODE
    validityMap =
      HoleValidation::validateHolesViaThresholding(
        probabilitiesVectorSomeRGBDMode, RGBD_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);



    ////////////////////////////// RGB_ONLY_MODE ///////////////////////////////

    /////////////////////////////// All filters ////////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgb_priority = 1;
    Parameters::Filters::LuminosityDiff::rgb_priority = 2;
    Parameters::Filters::TextureBackprojection::rgb_priority = 3;
    Parameters::Filters::TextureDiff::rgb_priority = 4;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Luminosity diff
    luminosityDiffValues.clear();
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Texture diff
    textureDiffValues.clear();
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorAllRGBOnlyMode.push_back( colourHomogeneityValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( luminosityDiffValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( textureBackprojectionValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( textureDiffValues );

    // Run HoleValidation::validateHolesViaThresholding for RGB_ONLY_MODE
    validityMap = HoleValidation::validateHolesViaThresholding(
        probabilitiesVectorAllRGBOnlyMode, RGB_ONLY_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);


    /////////////////////////////// Some filters ///////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgb_priority = 1;
    Parameters::Filters::LuminosityDiff::rgb_priority = 2;
    Parameters::Filters::TextureBackprojection::rgb_priority = 3;
    Parameters::Filters::TextureDiff::rgb_priority = 0;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Luminosity diff
    luminosityDiffValues.clear();
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorSomeRGBOnlyMode.push_back( colourHomogeneityValues );
    probabilitiesVectorSomeRGBOnlyMode.push_back( luminosityDiffValues );
    probabilitiesVectorSomeRGBOnlyMode.push_back( textureBackprojectionValues );

    // Run HoleValidation::validateHolesViaThresholding for RGB_ONLY_MODE
    validityMap = HoleValidation::validateHolesViaThresholding(
        probabilitiesVectorSomeRGBOnlyMode, RGB_ONLY_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);
  }



  //! Tests HoleValidation::validateHolesViaWeighting
  TEST_F ( HoleValidationTest, validateHolesViaWeightingTest )
  {
    //////////////////////////////// RGBD_MODE /////////////////////////////////

    //////////////////////////////// All filters ///////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 1;
    Parameters::Filters::DepthArea::priority = 2;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 3;
    Parameters::Filters::RectanglePlaneConstitution::priority = 4;
    Parameters::Filters::DepthHomogeneity::priority = 5;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 6;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 7;
    Parameters::Filters::DepthDiff::priority = 8;
    Parameters::Filters::TextureDiff::rgbd_priority = 9;


    // Set up filters' responses about two holes

    // Colour homogeneity
    std::vector< float > colourHomogeneityValues;
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Depth / area
    std::vector< float > depthAreaValues;
    depthAreaValues.push_back( 0.0 );
    depthAreaValues.push_back( 1.0 );

    // Luminosity diff
    std::vector< float > luminosityDiffValues;
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Rectangle plane constitution
    std::vector< float > rectanglePlaneConstitutionValues;
    rectanglePlaneConstitutionValues.push_back( 0.0 );
    rectanglePlaneConstitutionValues.push_back( 1.0 );

    // Depth homogeneity
    std::vector< float > depthHomogeneityValues;
    depthHomogeneityValues.push_back( 0.0 );
    depthHomogeneityValues.push_back( 1.0 );

    // Texture backprojection
    std::vector< float > textureBackprojectionValues;
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Intermediate points plane constitution
    std::vector< float > intermediatePointsPlaneConstitutionValues;
    intermediatePointsPlaneConstitutionValues.push_back( 0.0 );
    intermediatePointsPlaneConstitutionValues.push_back( 1.0 );

    // Depth diff
    std::vector< float > depthDiffValues;
    depthDiffValues.push_back( 0.0 );
    depthDiffValues.push_back( 1.0 );

    // Texture diff
    std::vector< float > textureDiffValues;
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorAllRGBDMode.push_back( colourHomogeneityValues );
    probabilitiesVectorAllRGBDMode.push_back( depthAreaValues );
    probabilitiesVectorAllRGBDMode.push_back( luminosityDiffValues );
    probabilitiesVectorAllRGBDMode.push_back( rectanglePlaneConstitutionValues );
    probabilitiesVectorAllRGBDMode.push_back( depthHomogeneityValues );
    probabilitiesVectorAllRGBDMode.push_back( textureBackprojectionValues );
    probabilitiesVectorAllRGBDMode.push_back( intermediatePointsPlaneConstitutionValues );
    probabilitiesVectorAllRGBDMode.push_back( depthDiffValues );
    probabilitiesVectorAllRGBDMode.push_back( textureDiffValues );


    // Run HoleValidation::validateHolesViaWeighting for RGBD_MODE
    std::map< int, float > validityMap =
      HoleValidation::validateHolesViaWeighting(
        probabilitiesVectorAllRGBDMode, RGBD_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    std::map< int, float>::iterator v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);


    /////////////////////////////// Some filters ///////////////////////////////

    // Set the priority of some available filters
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 1;
    Parameters::Filters::DepthArea::priority = 2;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 0;
    Parameters::Filters::RectanglePlaneConstitution::priority = 3;
    Parameters::Filters::DepthHomogeneity::priority = 0;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 4;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 5;
    Parameters::Filters::DepthDiff::priority = 6;
    Parameters::Filters::TextureDiff::rgbd_priority = 0;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Depth / area
    depthAreaValues.clear();
    depthAreaValues.push_back( 0.0 );
    depthAreaValues.push_back( 1.0 );

    // Rectangle plane constitution
    rectanglePlaneConstitutionValues.clear();
    rectanglePlaneConstitutionValues.push_back( 0.0 );
    rectanglePlaneConstitutionValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Intermediate points plane constitution
    intermediatePointsPlaneConstitutionValues.clear();
    intermediatePointsPlaneConstitutionValues.push_back( 0.0 );
    intermediatePointsPlaneConstitutionValues.push_back( 1.0 );

    // Texture diff
    textureDiffValues.clear();
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorSomeRGBDMode.push_back( colourHomogeneityValues );
    probabilitiesVectorSomeRGBDMode.push_back( depthAreaValues );
    probabilitiesVectorSomeRGBDMode.push_back( rectanglePlaneConstitutionValues );
    probabilitiesVectorSomeRGBDMode.push_back( textureBackprojectionValues );
    probabilitiesVectorSomeRGBDMode.push_back( intermediatePointsPlaneConstitutionValues );
    probabilitiesVectorSomeRGBDMode.push_back( textureDiffValues );


    // Run HoleValidation::validateHolesViaWeighting for RGBD_MODE
    validityMap =
      HoleValidation::validateHolesViaWeighting(
        probabilitiesVectorSomeRGBDMode, RGBD_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);



    ////////////////////////////// RGB_ONLY_MODE ///////////////////////////////

    /////////////////////////////// All filters ////////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgb_priority = 1;
    Parameters::Filters::LuminosityDiff::rgb_priority = 2;
    Parameters::Filters::TextureBackprojection::rgb_priority = 3;
    Parameters::Filters::TextureDiff::rgb_priority = 4;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Luminosity diff
    luminosityDiffValues.clear();
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Texture diff
    textureDiffValues.clear();
    textureDiffValues.push_back( 0.0 );
    textureDiffValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorAllRGBOnlyMode.push_back( colourHomogeneityValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( luminosityDiffValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( textureBackprojectionValues );
    probabilitiesVectorAllRGBOnlyMode.push_back( textureDiffValues );

    // Run HoleValidation::validateHolesViaWeighting for RGB_ONLY_MODE
    validityMap = HoleValidation::validateHolesViaWeighting(
        probabilitiesVectorAllRGBOnlyMode, RGB_ONLY_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);


    /////////////////////////////// Some filters ///////////////////////////////

    // Set the priority of all available filters
    Parameters::Filters::ColourHomogeneity::rgb_priority = 1;
    Parameters::Filters::LuminosityDiff::rgb_priority = 2;
    Parameters::Filters::TextureBackprojection::rgb_priority = 3;
    Parameters::Filters::TextureDiff::rgb_priority = 0;


    // Set up filters' responses about two holes

    // Colour homogeneity
    colourHomogeneityValues.clear();
    colourHomogeneityValues.push_back( 0.0 );
    colourHomogeneityValues.push_back( 1.0 );

    // Luminosity diff
    luminosityDiffValues.clear();
    luminosityDiffValues.push_back( 0.0 );
    luminosityDiffValues.push_back( 1.0 );

    // Texture backprojection
    textureBackprojectionValues.clear();
    textureBackprojectionValues.push_back( 0.0 );
    textureBackprojectionValues.push_back( 1.0 );

    // Populate the overall probabilities vector, for RGBD_MODE
    probabilitiesVectorSomeRGBOnlyMode.push_back( colourHomogeneityValues );
    probabilitiesVectorSomeRGBOnlyMode.push_back( luminosityDiffValues );
    probabilitiesVectorSomeRGBOnlyMode.push_back( textureBackprojectionValues );

    // Run HoleValidation::validateHolesViaWeighting for RGB_ONLY_MODE
    validityMap = HoleValidation::validateHolesViaWeighting(
        probabilitiesVectorSomeRGBOnlyMode, RGB_ONLY_MODE );

    // Only the second hole should have been deemed valid
    ASSERT_EQ ( 1, validityMap.size() );

    v_it = validityMap.begin();

    ASSERT_EQ ( 1, v_it->first);
    ASSERT_EQ ( 1.0, v_it->second);
  }

} // namespace pandora_vision

#include "utils/morphological_operators.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  class MorphologyTest : public ::testing::Test
  {
    protected:

      MorphologyTest() {}

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        img0_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
        img1_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
        img2_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        ASSERT_EQ( HEIGHT, img0_.rows );
        ASSERT_EQ( WIDTH, img0_.cols );

        // img0_ holds a horizontal white line at row 100,
        // with width of 1 pixel
        for ( int cols = 0; cols < img0_.cols; ++cols )
        {
          img0_.at<unsigned char>( 100, cols ) = 255;
        }

        // img1_ holds a single non-zero pixel
        img1_.at<unsigned char>( 100, 100 ) = 255;

        // img2_ is a white image except for its borders which are black
        for ( int rows = 1; rows < img2_.rows - 1; rows++ )
        {
          for ( int cols = 1; cols < img2_.cols - 1; cols++ )
          {
            img2_.at<unsigned char>(rows, cols) = 255;
          }
        }
      }

      int WIDTH;
      int HEIGHT;

      cv::Mat img0_;
      cv::Mat img1_;
      cv::Mat img2_;

  };



  //!< Test Morphology::dilation()
  TEST_F( MorphologyTest, DilationTest )
  {
    /***************************************************************************
    * Test img0_
    ***************************************************************************/

    // The number of non-zero pixels before dilation
    int nonZerosBefore = cv::countNonZero( img0_ );

    // Dilate once
    Morphology::dilation( &img0_, 1 );

    // The number of non-zero pixels after dilation
    int nonZerosAfter = cv::countNonZero( img0_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels before the dilation should be three times
    // as many as that of the pixels after
    EXPECT_EQ( 3 * nonZerosBefore, nonZerosAfter );

    // One row higher and one row lower than 100, all pixels should now
    // have a non-zero value
    for( int cols = 0; cols < img0_.cols; ++cols )
    {
      EXPECT_EQ( 255, img0_.at<unsigned char>( 99, cols ));
      EXPECT_EQ( 255, img0_.at<unsigned char>( 101, cols ));
    }


    /***************************************************************************
    * Test img1_
    ***************************************************************************/

    // Perform dilation on the img1_
    Morphology::dilation( &img1_, 1 );

    // All pixels surrounding immediately the only non-zero one before dilation
    // should now have a non-zero value
    for ( int i = -1; i < 2; ++i )
    {
      for ( int j = -1; j < 2; ++j )
      {
        EXPECT_EQ( 255, img1_.at<unsigned char>( 100 + i, 100 + j ));
      }
    }


    /***************************************************************************
     * Test img2_
     ***************************************************************************/

    // The number of non-zero pixels before dilation
    nonZerosBefore = cv::countNonZero( img2_ );

    // Dilate once
    Morphology::dilation( &img2_, 1 );

    // The number of non-zero pixels after dilation
    nonZerosAfter = cv::countNonZero( img2_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the dilation should be
    // the sum of the image's pixels
    EXPECT_EQ( WIDTH * HEIGHT, nonZerosAfter );
  }



  //!< Test Morphology::dilationRelative()
  TEST_F( MorphologyTest, DilationRelativeTest )
  {
    /***************************************************************************
    * Test img0_
    ***************************************************************************/

    // The number of non-zero pixels before dilationRelativeRelative
    int nonZerosBefore = cv::countNonZero( img0_ );

    // Dilate once
    Morphology::dilationRelative( &img0_, 1 );

    // The number of non-zero pixels after dilationRelative
    int nonZerosAfter = cv::countNonZero( img0_ );

    // The number of non-zero pixels before the dilationRelative should be
    // less than that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels before the dilationRelative should be
    // three times as many as that of the pixels after
    EXPECT_EQ( 3 * nonZerosBefore, nonZerosAfter );

    // One row higher and one row lower than 100, all pixels should now
    // have a non-zero value
    for( int cols = 0; cols < img0_.cols; ++cols )
    {
      EXPECT_EQ( 255, img0_.at<unsigned char>( 99, cols ));
      EXPECT_EQ( 255, img0_.at<unsigned char>( 101, cols ));
    }


    /***************************************************************************
    * Test img1_
    ***************************************************************************/

    // Perform dilationRelative on the dilationRelative1_
    Morphology::dilationRelative( &img1_, 1 );

    // All pixels surrounding immediately the only non-zero one before
    // dilationRelative should now have a non-zero value
    for ( int i = -1; i < 2; ++i )
    {
      for ( int j = -1; j < 2; ++j )
      {
        EXPECT_EQ( 255, img1_.at<unsigned char>( 100 + i, 100 + j ));
      }
    }


    /***************************************************************************
     * Test img2_
     ***************************************************************************/

    // The number of non-zero pixels before dilationRelative
    nonZerosBefore = cv::countNonZero( img2_ );

    // Dilate once
    Morphology::dilationRelative( &img2_, 1 );

    // The number of non-zero pixels after dilationRelative
    nonZerosAfter = cv::countNonZero( img2_ );

    // The number of non-zero pixels before the dilationRelative should be
    // less than that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the dilationRelative should be
    // the sum of the image's pixels
    EXPECT_EQ( WIDTH * HEIGHT, nonZerosAfter );
  }



  //!< Test Morphology::erosion()
  TEST_F( MorphologyTest, ErosionTest )
  {
    /***************************************************************************
    * Test img0_
    ***************************************************************************/

    // The number of non-zero pixels before erosion
    int nonZerosBefore = cv::countNonZero( img0_ );

    // Erode once
    Morphology::erosion( &img0_, 1 );

    // The number of non-zero pixels after erosion
    int nonZerosAfter = cv::countNonZero( img0_ );

    // The number of non-zero pixels before the erosion should be
    // less than that of the pixels after
    EXPECT_GT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the erosion should be exactly zero
    EXPECT_EQ( 0, nonZerosAfter );


    /***************************************************************************
     * Test img2_
     ***************************************************************************/

    // The number of non-zero pixels before erosion
    nonZerosBefore = cv::countNonZero( img2_ );

    // Erode once
    Morphology::erosion( &img2_, 1 );

    // The number of non-zero pixels after erosion
    nonZerosAfter = cv::countNonZero( img2_ );

    // The number of non-zero pixels before the erosion should be
    // greater than that of the pixels after
    EXPECT_GT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the erosion should be
    // three times as many as that of the pixels after
    EXPECT_EQ( nonZerosBefore - 2 * WIDTH -2 * HEIGHT + 4, nonZerosAfter );
  }

}  // namespace pandora_vision

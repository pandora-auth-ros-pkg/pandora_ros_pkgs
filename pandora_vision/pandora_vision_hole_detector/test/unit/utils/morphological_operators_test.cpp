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

        dilation0_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
        dilation1_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
        dilation2_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        ASSERT_EQ( HEIGHT, dilation0_.rows );
        ASSERT_EQ( WIDTH, dilation0_.cols );

        // dilation0_ holds a horizontal white line at row 100,
        // with width of 1 pixel
        for ( int cols = 0; cols < dilation0_.cols; ++cols )
        {
          dilation0_.at<unsigned char>( 100, cols ) = 255;
        }

        // dilation1_ holds a single non-zero pixel
        dilation1_.at<unsigned char>( 100, 100 ) = 255;

        // dilation2_ is a white image except for its borders which are black
        for ( int rows = 1; rows < dilation2_.rows - 1; rows++ )
        {
          for ( int cols = 1; cols < dilation2_.cols - 1; cols++ )
          {
            dilation2_.at<unsigned char>(rows, cols) = 255;
          }
        }
      }

      int WIDTH;
      int HEIGHT;

      cv::Mat dilation0_;
      cv::Mat dilation1_;
      cv::Mat dilation2_;

  };



  //!< Test Morphology::dilation()
  TEST_F( MorphologyTest, DilationTest )
  {
    /***************************************************************************
    * Test dilation0_
    ***************************************************************************/

    // The number of non-zero pixels before dilation
    int nonZerosBefore = cv::countNonZero( dilation0_ );

    // Dilate once
    Morphology::dilation( &dilation0_, 1 );

    // The number of non-zero pixels after dilation
    int nonZerosAfter = cv::countNonZero( dilation0_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels before the dilation should be three times
    // as many as that of the pixels after
    EXPECT_EQ( 3 * nonZerosBefore, nonZerosAfter );

    // One row higher and one row lower than 100, all pixels should now
    // have a non-zero value
    for( int cols = 0; cols < dilation0_.cols; ++cols )
    {
      EXPECT_EQ( 255, dilation0_.at<unsigned char>( 99, cols ));
      EXPECT_EQ( 255, dilation0_.at<unsigned char>( 101, cols ));
    }

    /***************************************************************************
    * Test dilation1_
    ***************************************************************************/

    // Perform dilation on the dilation1_
    Morphology::dilation( &dilation1_, 1 );

    // All pixels surrounding immediately the only non-zero one before dilation
    // should now have a non-zero value
    for ( int i = -1; i < 2; ++i )
    {
      for ( int j = -1; j < 2; ++j )
      {
        EXPECT_EQ( 255, dilation1_.at<unsigned char>( 100 + i, 100 + j ));
      }
    }

    /***************************************************************************
     * Test dilation2_
     ***************************************************************************/

    // The number of non-zero pixels before dilation
    nonZerosBefore = cv::countNonZero( dilation2_ );

    // Dilate once
    Morphology::dilation( &dilation2_, 1 );

    // The number of non-zero pixels after dilation
    nonZerosAfter = cv::countNonZero( dilation2_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels before the dilation should be three times
    // as many as that of the pixels after
    EXPECT_EQ( WIDTH * HEIGHT, nonZerosAfter );
  }

}  // namespace pandora_vision

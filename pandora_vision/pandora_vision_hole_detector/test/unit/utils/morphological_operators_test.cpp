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
      }

      int WIDTH;
      int HEIGHT;

      cv::Mat img0_;
      cv::Mat img1_;

  };



  //!< Test Morphology::dilation()
  TEST_F( MorphologyTest, DilationTest )
  {
    // The number of non-zero pixels before dilation
    int nonZerosBefore = cv::countNonZero( img0_ );

    // Dilate once
    Morphology::dilation( &img0_, 1 );

    // The number of non-zero pixels after dilation
    int nonZerosAfter = cv::countNonZero( img0_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT(nonZerosBefore, nonZerosAfter);

    // One row higher and one row lower than 100, all pixels should now
    // have a non-zero value
    for( int i = 1; i < img0_.cols - 1; ++i )
    {
      EXPECT_EQ( 255, img0_.at<unsigned char>( 99, i ));
      EXPECT_EQ( 255, img0_.at<unsigned char>( 101, i ));
    }

    EXPECT_EQ( 0, img0_.at<unsigned char>( 99, 0 ));
    EXPECT_EQ( 0, img0_.at<unsigned char>( 101, 0 ));
    EXPECT_EQ( 0, img0_.at<unsigned char>( 99, WIDTH - 1 ));
    EXPECT_EQ( 0, img0_.at<unsigned char>( 101, WIDTH - 1 ));

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
  }

}  // namespace pandora_vision


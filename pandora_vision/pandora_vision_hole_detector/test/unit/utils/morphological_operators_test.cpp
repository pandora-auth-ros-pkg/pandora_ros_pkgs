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

        line_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
        smallSquare_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
        bigSquare_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        ASSERT_EQ( HEIGHT, line_.rows );
        ASSERT_EQ( WIDTH, line_.cols );

        // line_ holds a horizontal white line at row 100,
        // with width of 1 pixel
        for ( int cols = 0; cols < line_.cols; ++cols )
        {
          line_.at<unsigned char>( 100, cols ) = 255;
        }

        // smallSquare_ holds a single non-zero pixel
        smallSquare_.at<unsigned char>( 100, 100 ) = 255;

        // bigSquare_ is a white image except for its borders which are black
        for ( int rows = 1; rows < bigSquare_.rows - 1; rows++ )
        {
          for ( int cols = 1; cols < bigSquare_.cols - 1; cols++ )
          {
            bigSquare_.at<unsigned char>(rows, cols) = 255;
          }
        }
      }

      int WIDTH;
      int HEIGHT;

      cv::Mat line_;
      cv::Mat smallSquare_;
      cv::Mat bigSquare_;

  };



  //!< Test Morphology::dilation()
  TEST_F( MorphologyTest, DilationTest )
  {
    /***************************************************************************
     * Test line_
     **************************************************************************/

    // The number of non-zero pixels before dilation
    int nonZerosBefore = cv::countNonZero( line_ );

    // Dilate once
    Morphology::dilation( &line_, 1 );

    // The number of non-zero pixels after dilation
    int nonZerosAfter = cv::countNonZero( line_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels before the dilation should be three times
    // as many as that of the pixels after
    EXPECT_EQ( 3 * nonZerosBefore, nonZerosAfter );

    // One row higher and one row lower than 100, all pixels should now
    // have a non-zero value
    for( int cols = 0; cols < line_.cols; ++cols )
    {
      EXPECT_EQ( 255, line_.at<unsigned char>( 99, cols ));
      EXPECT_EQ( 255, line_.at<unsigned char>( 101, cols ));
    }


    /***************************************************************************
     * Test smallSquare_
     **************************************************************************/

    // Perform dilation on the smallSquare_
    Morphology::dilation( &smallSquare_, 1 );

    // All pixels surrounding immediately the only non-zero one before dilation
    // should now have a non-zero value
    for ( int i = -1; i < 2; ++i )
    {
      for ( int j = -1; j < 2; ++j )
      {
        EXPECT_EQ( 255, smallSquare_.at<unsigned char>( 100 + i, 100 + j ));
      }
    }


    /***************************************************************************
     * Test bigSquare_
     **************************************************************************/

    // The number of non-zero pixels before dilation
    nonZerosBefore = cv::countNonZero( bigSquare_ );

    // Dilate once
    Morphology::dilation( &bigSquare_, 1 );

    // The number of non-zero pixels after dilation
    nonZerosAfter = cv::countNonZero( bigSquare_ );

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
     * Test line_
     **************************************************************************/

    // The number of non-zero pixels before dilationRelativeRelative
    int nonZerosBefore = cv::countNonZero( line_ );

    // Dilate once
    Morphology::dilationRelative( &line_, 1 );

    // The number of non-zero pixels after dilationRelative
    int nonZerosAfter = cv::countNonZero( line_ );

    // The number of non-zero pixels before the dilationRelative should be
    // less than that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels before the dilationRelative should be
    // three times as many as that of the pixels after
    EXPECT_EQ( 3 * nonZerosBefore, nonZerosAfter );

    // One row higher and one row lower than 100, all pixels should now
    // have a non-zero value
    for( int cols = 0; cols < line_.cols; ++cols )
    {
      EXPECT_EQ( 255, line_.at<unsigned char>( 99, cols ));
      EXPECT_EQ( 255, line_.at<unsigned char>( 101, cols ));
    }


    /***************************************************************************
     * Test smallSquare_
     **************************************************************************/

    // Perform dilationRelative on the dilationRelative1_
    Morphology::dilationRelative( &smallSquare_, 1 );

    // All pixels surrounding immediately the only non-zero one before
    // dilationRelative should now have a non-zero value
    for ( int i = -1; i < 2; ++i )
    {
      for ( int j = -1; j < 2; ++j )
      {
        EXPECT_EQ( 255, smallSquare_.at<unsigned char>( 100 + i, 100 + j ));
      }
    }


    /***************************************************************************
     * Test bigSquare_
     **************************************************************************/

    // The number of non-zero pixels before dilationRelative
    nonZerosBefore = cv::countNonZero( bigSquare_ );

    // Dilate once
    Morphology::dilationRelative( &bigSquare_, 1 );

    // The number of non-zero pixels after dilationRelative
    nonZerosAfter = cv::countNonZero( bigSquare_ );

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
     * Test line_
     **************************************************************************/

    // The number of non-zero pixels before erosion
    int nonZerosBefore = cv::countNonZero( line_ );

    // Erode once
    Morphology::erosion( &line_, 1 );

    // The number of non-zero pixels after erosion
    int nonZerosAfter = cv::countNonZero( line_ );

    // The number of non-zero pixels before the erosion should be
    // less than that of the pixels after
    EXPECT_GT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the erosion should be exactly zero
    EXPECT_EQ( 0, nonZerosAfter );


    /***************************************************************************
     * Test bigSquare_
     **************************************************************************/

    // The number of non-zero pixels before erosion
    nonZerosBefore = cv::countNonZero( bigSquare_ );

    // Erode once
    Morphology::erosion( &bigSquare_, 1 );

    // The number of non-zero pixels after erosion
    nonZerosAfter = cv::countNonZero( bigSquare_ );

    // The number of non-zero pixels before the erosion should be
    // greater than that of the pixels after
    EXPECT_GT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the erosion should be
    // three times as many as that of the pixels after
    EXPECT_EQ( nonZerosBefore - 2 * (WIDTH - 2) -2 * (HEIGHT - 2) + 4,
      nonZerosAfter );
  }

}  // namespace pandora_vision

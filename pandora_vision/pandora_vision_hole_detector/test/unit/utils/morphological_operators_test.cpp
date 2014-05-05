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
        WIDTH = 100;
        HEIGHT = 100;
        img_ = cv::Mat::zeros(WIDTH, HEIGHT, CV_8UC1);
        img2_ = cv::Mat::zeros(WIDTH, HEIGHT, CV_8UC1);
        ASSERT_EQ(HEIGHT, img_.rows);
        ASSERT_EQ(WIDTH, img_.cols);
        for(int i = 0; i < img_.cols; ++i)
        {
          img_.at<unsigned char>(50, i) = 255;
        }
        img2_.at<unsigned char>(50, 50) = 255;
      }

      cv::Mat img_;
      cv::Mat img2_;
      int WIDTH;
      int HEIGHT;
  };

  TEST_F(MorphologyTest, DilationTest)
  {
    int nonZerosBefore = cv::countNonZero(img_);
    Morphology::dilation(&img_, 1);
    int nonZerosAfter = cv::countNonZero(img_);
    EXPECT_LT(nonZerosBefore, nonZerosAfter);
    for(int i = 1; i < img_.cols - 1; ++i)
    {
      EXPECT_EQ( 255, img_.at<unsigned char>(49, i));
      EXPECT_EQ( 255, img_.at<unsigned char>(51, i));
    }
    //EXPECT_EQ( 255, img_.at<unsigned char>(49, 0));
    //EXPECT_EQ( 255, img_.at<unsigned char>(51, 0));
    //EXPECT_EQ( 255, img_.at<unsigned char>(49, WIDTH-1));
    //EXPECT_EQ( 255, img_.at<unsigned char>(51, WIDTH-1));

    Morphology::dilation(&img2_, 1);
    for(int i = -1; i < 2; ++i) {
      for(int j = -1; j < 2; ++j) {
        EXPECT_EQ( 255, img2_.at<unsigned char>(50 + i, 50 + j));
      }
    }
  }

}  // namespace pandora_vision


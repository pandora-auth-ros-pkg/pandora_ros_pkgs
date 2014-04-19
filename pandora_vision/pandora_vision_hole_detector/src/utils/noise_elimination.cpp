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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#include "utils/noise_elimination.h"

namespace pandora_vision
{
  /**
    @brief Interpolates the noise produced by the depth sensor.
    The black blobs take the depth value of the closest neighbour obstacles.
    @param[in] inImage [const cv::Mat&] The input image
    @param[out] outImage [cv::Mat*] The output image
    @return void
   **/
  void NoiseElimination::brushfireNear(const cv::Mat& inImage,
    cv::Mat* outImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("brushfireNear", "performNoiseElimination");
    #endif

    inImage.copyTo(*outImage);
    bool finished = false;

    //!< Assign the image borders a non zero value
    for(unsigned int i = 0 ; i < outImage->rows ; i++)
    {
      outImage->at<float>(i, 0) = 1;
      outImage->at<float>(i, outImage->cols - 1) = 1;
    }

    for(unsigned int i = 0 ; i < outImage->cols ; i++)
    {
      outImage->at<float>(0, i) = 1;
      outImage->at<float>(outImage->rows - 1, i) = 1;
    }

    while(!finished)
    {
      finished = true;

      for(unsigned int i = 1 ; i < outImage->rows - 1 ; i++)
      {
        for(unsigned int j = 1 ; j< outImage->cols - 1 ; j++)
        {
          if(outImage->at<float>(i, j) == 0.0)  //!< Found black
          {
            brushfireNearStep(outImage, i * outImage->cols + j);
            finished = false;
            break;
          }
        }

        if(!finished)
        {
          break;
        }
      }
    }
    #ifdef DEBUG_TIME
    Timer::tick("brushfireNear");
    #endif
  }



  /**
    @brief Iteration for the interpolateNoise_brushNear function
    @param[in][out] image [cv::Mat&] The input image
    @param index [const int&] Where to start the brushfire algorithm
    (index = y * cols + x)
    @return void
   **/
  void NoiseElimination::brushfireNearStep(cv::Mat* image,
    const int& index)
  {
    #ifdef DEBUG_TIME
    Timer::start("brushfireNearStep");
    #endif

    unsigned int x = index / image->cols;
    unsigned int y = index % image->cols;

    std::vector<unsigned int> current, next;
    std::set<unsigned int> visited;

    current.push_back(x * image->cols + y);
    visited.insert(x * image->cols + y);

    while(current.size() != 0)
    {
      for(unsigned int i = 0; i < current.size(); i++)
      {
        for(int m = -1 ; m < 2 ; m++)
        {
          for(int n = -1 ; n < 2 ; n++)
          {
            x = static_cast<int>(current[i]) / image->cols + m;
            y = static_cast<int>(current[i]) % image->cols + n;

            if((image->at<float>(x, y) == 0) &&
              visited.find(x * image->cols + y) == visited.end())
            {
              next.push_back(x * image->cols + y);
              visited.insert(x * image->cols + y);
            }
          }
        }

      }
      current.swap(next);
      next.clear();
    }

    float lower = 10000.0;
    float val = 0;

    for(std::set<unsigned int>::iterator it = visited.begin();
      it != visited.end(); it++)
    {
      x = *it / image->cols;
      y = *it % image->cols;

      val = image->at<float>(x-1, y+1);
      if(val < lower && val != 0) lower = val;

      val = image->at<float>(x-1, y-1);
      if(val < lower && val != 0) lower = val;

      val = image->at<float>(x-1, y);
      if(val < lower && val != 0) lower = val;

      val = image->at<float>(x+1, y+1);
      if(val < lower && val != 0) lower = val;

      val = image->at<float>(x+1, y-1);
      if(val < lower && val != 0) lower = val;

      val = image->at<float>(x+1, y);
      if(val < lower && val != 0) lower = val;

      val = image->at<float>(x, y-1);
      if(val < lower && val != 0) lower = val;

      val = image->at<float>(x, y+1);
      if(val < lower && val != 0) lower = val;
    }

    for(std::set<unsigned int>::iterator it = visited.begin();
      it != visited.end(); it++)
    {
      image->at<float>(*it / image->cols, *it % image->cols) = lower;
    }

    #ifdef DEBUG_TIME
    Timer::tick("brushfireNearStep");
    #endif
  }



  /**
    @brief Changes the interpolation method according to the image's values
    @param[in] image [const cv::Mat&] The input image
    @return void
   **/
  void NoiseElimination::chooseInterpolationMethod(const cv::Mat& image)
  {
    #ifdef DEBUG_TIME
    Timer::start("chooseInterpolationMethod");
    #endif

    unsigned int blacks = 0;
    float mean = 0;
    float std = 0;
    float bper = 0;

    for(unsigned int i = 0 ; i < image.rows ; i++)
    {
      for(unsigned int j = 0 ; j < image.cols ; j++)
      {
        float d = image.at<float>(i, j);
        if(d == 0)
        {
          blacks++;
        }
        else
        {
          mean += d;
        }
      }
    }

    mean /= image.rows * image.cols - blacks + 1;

    for(unsigned int i = 0 ; i < image.rows ; i++)
    {
      for(unsigned int j = 0 ; j < image.cols ; j++)
      {
        float d = image.at<float>(i, j);
        if(d != 0)
        {
          std += pow(d - mean, 2);
        }
      }
    }

    std /= image.rows * image.cols - blacks + 1;
    std = sqrt(std);
    bper = static_cast<float>(blacks) / (image.rows * image.cols);

    Parameters::interpolation_method = 15;
    if(bper > 0.7)  //!< Choose close
    {
      Parameters::interpolation_method = 2;
    }
    else if(mean < 0.7)
    {
      Parameters::interpolation_method = 1;
    }
    else
    {
      Parameters::interpolation_method = 0;
    }
    #ifdef DEBUG_TIME
    Timer::tick("chooseInterpolationMethod");
    #endif
  }



  /**
    @brief Interpolates the noise of an image at its borders.
    @param[in][out] inImage [cv::Mat*] The image whose noise will be
    interpolated at the edges.
    @return void
   **/
  void NoiseElimination::interpolateImageBorders(cv::Mat* inImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("interpolateImageBorders");
    #endif

    //!< interpolate the pixels at the edges of the inImage
    //!< interpolate the rows
    for (unsigned int i = 1; i < inImage->cols - 1; ++i)
    {
      inImage->at<float>(0, i) = inImage->at<float>(1, i);
      inImage->at<float>(inImage->rows - 1, i) =
        inImage->at<float>(inImage->rows - 2, i);
    }

    //!< interpolate the columns
    for (unsigned int i = 1; i < inImage->rows - 1; ++i)
    {
      inImage->at<float>(i, 0) = inImage->at<float>(i, 1);
      inImage->at<float>(i, inImage->cols - 1) =
        inImage->at<float>(i, inImage->cols - 2);
    }

    //!< interpolate the corners
    /// top left
    inImage->at<float>(0, 0) = inImage->at<float>(1, 1);
    /// top right
    inImage->at<float>(0, inImage->cols - 1) =
      inImage->at<float>(1, inImage->cols - 2);
    /// bottom left
    inImage->at<float>(inImage->rows - 1, 0) =
      inImage->at<float>(inImage->rows - 2, 1);
    /// bottom right
    inImage->at<float>(inImage->rows - 1, inImage->cols - 1) =
      inImage->at<float>(inImage->rows - 2, inImage->cols - 2);

    #ifdef DEBUG_TIME
    Timer::tick("interpolateImageBorders");
    #endif
  }



  /**
    @brief Replaces the value (0) of pixel im(row, col) with the mean value
    of its non-zero neighbors.
    @param[in] inImage [const cv::Mat &] The input depth image
    @param[in] row [const int&] The row index of the pixel of interest
    @param[in] col [const int&] The column index of the pixel of interest
    @param[in][out] endFlag [bool*] True indicates that there are pixels
    left with zero value
    @return void
   **/
  float NoiseElimination::interpolateZeroPixel(
    const cv::Mat& inImage,
    const int& row,
    const int& col,
    bool* endFlag)
  {
    float sumValueOfNonZeroPixels = 0;
    int countOfNonZeroPixels = 0;

    float p2 = inImage.at<float>(row - 1, col);
    float p3 = inImage.at<float>(row - 1, col + 1);
    float p4 = inImage.at<float>(row, col + 1);
    float p5 = inImage.at<float>(row + 1, col + 1);
    float p6 = inImage.at<float>(row + 1, col);
    float p7 = inImage.at<float>(row + 1, col - 1);
    float p8 = inImage.at<float>(row, col - 1);
    float p9 = inImage.at<float>(row - 1, col - 1);

    if (p2 != 0)
    {
      sumValueOfNonZeroPixels += p2;
      countOfNonZeroPixels++;
      *endFlag = true;
    }
    if (p3 != 0)
    {
      sumValueOfNonZeroPixels += p3;
      countOfNonZeroPixels++;
      *endFlag = true;
    }
    if (p4 != 0)
    {
      sumValueOfNonZeroPixels += p4;
      countOfNonZeroPixels++;
      *endFlag = true;
    }
    if (p5 != 0)
    {
      sumValueOfNonZeroPixels += p5;
      countOfNonZeroPixels++;
      *endFlag = true;
    }
    if (p6 != 0)
    {
      sumValueOfNonZeroPixels += p6;
      countOfNonZeroPixels++;
      *endFlag = true;
    }
    if (p7 != 0)
    {
      sumValueOfNonZeroPixels += p7;
      countOfNonZeroPixels++;
      *endFlag = true;
    }
    if (p8 != 0)
    {
      sumValueOfNonZeroPixels += p8;
      countOfNonZeroPixels++;
      *endFlag = true;
    }
    if (p9 != 0)
    {
      sumValueOfNonZeroPixels += p9;
      countOfNonZeroPixels++;
      *endFlag = true;
    }
    if (countOfNonZeroPixels > 0)
    {
      return (sumValueOfNonZeroPixels / countOfNonZeroPixels);
    }
    return 0;
  }



  /**
    @brief Interpolates the noise produced by kinect. The noise is the areas\
    kinect cannot produce depth measurements (value = 0)
    @param[in] inImage [const cv::Mat&] The input image
    @param[out] outImage [cv::Mat*] The output image
    @return void
   **/
  void NoiseElimination::interpolation(const cv::Mat& inImage,
    cv::Mat* outImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("interpolation", "performNoiseElimination");
    #endif

    inImage.copyTo(*outImage);
    //!< in the end, only pixels adjacent to the edge of the
    //!< image are left black
    while (interpolationIteration(outImage)){}

    #ifdef DEBUG_TIME
    Timer::tick("interpolation");
    #endif
  }



  /**
    @brief Iteration for the interpolateNoise function
    @param[in][out] inImage [cv::Mat*] The input image
    @return flag [bool]. if flag == true there exist pixels
    that their value needs to be interpolated
   **/
  bool NoiseElimination::interpolationIteration(cv::Mat* inImage)
  {
    cv::Mat marker;
    inImage->copyTo(marker);

    bool flag = false;

    for (int i = 1; i < inImage->rows - 1; i++)
    {
      for (int j = 1; j < inImage->cols - 1; j++)
      {
        if (inImage->at<float>(i, j) == 0)
        {
          marker.at<float>(i, j) =
            interpolateZeroPixel(*inImage, i, j, &flag);
        }
      }
    }

    *inImage = marker;

    return flag;
  }



  /**
    @brief Given an input image from the depth sensor, this function
    eliminates noise in it depending on the amount of noise
    @param[in] inImage [const cv::Mat&] The input image
    @param[out] outImage [cv::Mat*] The denoised depth image
    @return void
   **/
  void NoiseElimination::performNoiseElimination(const cv::Mat& inImage,
    cv::Mat* outImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("performNoiseElimination", "findHoles");
    #endif

    chooseInterpolationMethod(inImage);

    switch(Parameters::interpolation_method)
    {
      case 0: //!< Thinning-like interpolation
        {
          interpolation(inImage, outImage);
          break;
        }
      case 1: //!< Produce the near brushfire image
        {
          brushfireNear(inImage, outImage);
          break;
        }
      case 2: //!< Produce the white noise image
        {
          transformNoiseToWhite(inImage, outImage);
          break;
        }
    }
    #ifdef DEBUG_TIME
    Timer::tick("performNoiseElimination");
    #endif
  }



  /**
    @brief Transforms all black noise to white
    @param[in] inImage [const cv::Mat&] The input image
    @param[out] outImage [cv::Mat*] The output image
    @return void
   **/
  void NoiseElimination::transformNoiseToWhite(const cv::Mat& inImage,
    cv::Mat* outImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("transformNoiseToWhite", "performNoiseElimination");
    #endif

    inImage.copyTo(*outImage);

    for(unsigned int i = 0 ; i < inImage.rows ; i++)
    {
      for(unsigned int j = 0 ; j < inImage.cols ; j++)
      {
        if(inImage.at<float>(i, j) != 0)
        {
          outImage->at<float>(i, j) = 4.0;
        }
      }
    }
    #ifdef DEBUG_TIME
    Timer::tick("transformNoiseToWhite");
    #endif
  }

} // namespace pandora_vision

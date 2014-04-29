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
 * Authors: Victor Daropoulos, Alexandros Philotheou
 *********************************************************************/

#include "utils/wavelets.h"

namespace pandora_vision
{

  Wavelets::Wavelets() {}



  std::vector<float> Wavelets::getH0(int index)
  {
    std::vector<float> h0;

    switch (index)
    {

      case 1: //Haar

        for(int i = 0; i < 2; i++)
        {
          h0.push_back(1 / sqrt(2));
        }

        break;

      case 2: //Daubechies 1

        h0.push_back(0.7071067812);
        h0.push_back(0.7071067812);
        break;

      case 3: //Daubechies 2

        h0.push_back(-0.1294095226);
        h0.push_back(0.2241438680);
        h0.push_back(0.8365163037);
        h0.push_back(0.4829629131);
        break;

      case 4: //Daubechies 3

        h0.push_back(0.0352262919);
        h0.push_back(-0.0854412739);
        h0.push_back(-0.1350110200);
        h0.push_back(0.4598775021);
        h0.push_back(0.8068915093);
        h0.push_back(0.3326705530);
        break;

      case 5: //Daubechies 4

        h0.push_back(-0.0105974018);
        h0.push_back(0.0328830117);
        h0.push_back(0.0308413818);
        h0.push_back(-0.1870348117);
        h0.push_back(-0.0279837694);
        h0.push_back(0.6308807679);
        h0.push_back(0.7148465706);
        h0.push_back(0.2303778133);
        break;
    }

    return h0;
  }



  std::vector<float> Wavelets::getG0(const std::vector<float>& H0)
  {
    std::vector<float> G0;

    for(int i = 0; i < H0.size(); i++)
    {
      G0.push_back(H0.at(i));
    }

    std::reverse(G0.begin(), G0.end());

    return G0;
  }



  std::vector<float> Wavelets::getG1(const std::vector<float>& H0)
  {

    std::vector<float> G1;

    for(int i = 0; i < H0.size(); i++)
    {
      int temp = pow(-1, i);

      G1.push_back(temp * H0.at(i));
    }

    return G1;
  }



  std::vector<float> Wavelets::getH1(const std::vector<float>& G1)
  {
    std::vector<float> H1;

    for(int i = 0; i < G1.size(); i++)
    {
      H1.push_back(G1.at(i));
    }

    std::reverse(H1.begin(), H1.end());

    return H1;
  }



  cv::Mat Wavelets::convCols(const cv::Mat& in,
    const std::vector<float>& kernel)
  {
    int length = in.rows + kernel.size() - 1;

    //std::cout<<"Length is : "<<length<<std::endl;

    cv::Mat temp = cv::Mat::zeros(length, in.cols, CV_32FC1);

    for(int k = 0; k < in.cols; k++)
    {
      for(int i = 0; i < length; i++)
      {
        float y = 0;

        for(int j = 0; j < kernel.size(); j++)
        {
          if((i - j) >= 0 && (i - j) < in.rows)
          {
            y += in.at<float>(i - j, k) * kernel.at(j);

          }

        }
        temp.at<float>(i, k)=y;
      }

      //out.push_back(y);
    }

    return temp;
  }



  cv::Mat Wavelets::convRows(const cv::Mat& in,
    const std::vector<float>& kernel)
  {
    int length = in.cols + kernel.size() - 1;

    cv::Mat temp = cv::Mat::zeros(in.rows, length, CV_32FC1);

    for(int k = 0; k < in.rows; k++)
    {
      for(int i = 0; i < length; i++)
      {
        float y = 0;

        for(int j = 0; j < kernel.size(); j++)
        {
          if((i - j) >= 0 && (i - j) < in.cols)
          {
            y += in.at<float>(k, i - j) * kernel.at(j);
          }

        }

        temp.at<float>(k, i)=y;
      }
    }

    return temp;
  }



  cv::Mat Wavelets::getLowLow(const cv::Mat& in,
    const std::vector<float>& kernel)
  {
    cv::Mat temp0 = convCols(in, kernel);

    cv::Mat tempy0;

    for(int i = 0; i < temp0.rows; i += 2)
    {
      tempy0.push_back(temp0.row(i));
    }

    cv::Mat tempy00 = convRows(tempy0, kernel);

    cv::Mat tempy000;

    cv::transpose(tempy00, tempy00);

    for(int i = 0; i < tempy00.rows; i += 2)
    {
      tempy000.push_back(tempy00.row(i));
    }

    cv::transpose(tempy000, tempy000);

    double minVal;
    double maxVal;


    cv::minMaxLoc(tempy000, &minVal, &maxVal);

    tempy000 = tempy000 / maxVal;

    return tempy000;
  }


  /**
    @brief Returns a CV_32FC1 image containing the low-low part of the
    input image, which is also in CV_32FC1 format
    @param[in] inImage [const cv::Mat&] The input image in CV_32FC1 format
    @param[in] min [const double&] The inImage's min value
    @param[in] max [const double&] The inImage's max value
    @param[out] outImage [cv::Mat*] The low-low part of the inImage in
    CV_32FC1 format
    @return void
   **/
  void Wavelets::getLowLow(const cv::Mat& inImage,
    const double& min, const double& max,
    cv::Mat* outImage)
  {
    cv::Mat temp = cv::Mat(inImage.size(), CV_8UC1);

    temp = Visualization::scaleImageForVisualization(inImage, 0);

    Wavelets* wave = new Wavelets();

    std::vector<float> H0 = wave->getH0(1);

    std::vector<float> G0 = wave->getG0(H0);

    std::vector<float> G1 = wave->getG1(H0);

    std::vector<float> H1 = wave->getH1(G1);


    cv::Mat doubled = cv::Mat::zeros(temp.rows, temp.cols, CV_32FC1);

    for(int y = 0; y < doubled.rows; y++)
    {
      for(int x = 0; x < doubled.cols; x++)
      {
        doubled.at<float>(y, x) =
          static_cast<float>(temp.at<unsigned char>(y, x) / 255.0);
      }
    }

    //!< LowLow contains the inImage's low low frequencies, in CV_32FC1 format
    //!< What we will return will be this image scaled to the actual proportions
    //!< of values of the inImage (also in CV_32FC1 format).
    //!< After obtaining the low-low, reverse the scale operation, in an
    //!< attempt to approximate the initial depth image's values
    *outImage = wave->getLowLow(doubled, H0) * (max - min);
  }



  /**
    @brief Returns a RGB CV_8UC3 image containing the low-low part of the
    input RGB image, which is also in CV_8UC3 format
    @param[in] inImage [const cv::Mat&] The input image in CV_8UC3 format
    @param[out] outImage [cv::Mat*] The low-low part of the inImage in
    CV_8UC3 format
    @return void
   **/
  void Wavelets::getLowLow(const cv::Mat& inImage, cv::Mat* outImage)
  {
    Wavelets* wave = new Wavelets();

    std::vector<float> H0 = wave->getH0(1);

    std::vector<float> G0 = wave->getG0(H0);

    std::vector<float> G1 = wave->getG1(H0);

    std::vector<float> H1 = wave->getH1(G1);


    cv::Mat lowLow[3];

    for (int i = 0; i < 3; i++)
    {
      cv::Mat doubled = cv::Mat::zeros(inImage.rows, inImage.cols, CV_32FC1);
      for(int y = 0; y < doubled.rows; y++)
      {
        for(int x = 0; x < doubled.cols; x++)
        {
          doubled.at<float>(y, x) =
            inImage.at<unsigned char>(y, 3 * x + i) / 255.0;
        }
      }

      //!< LowLow contains the inImage's low low frequencies, in CV_32FC1 format
      //!< What we will return will be this image scaled to the actual
      //!< proportions of values of the inImage (also in CV_32FC1 format).
      lowLow[i] = wave->getLowLow(doubled, H0);
    }

    //!< The outImage. out has to be assigned to *outImage but cannot be done
    //!< in the following loops immediately
    cv::Mat out = cv::Mat::zeros(lowLow[0].size(), CV_8UC3);

    for (int i = 0; i < 3; i++)
    {
      for(int y = 0; y < lowLow[i].rows; y++)
      {
        for(int x = 0; x < lowLow[i].cols; x++)
        {
          out.at<unsigned char>(y, 3 * x + i) = lowLow[i].at<float>(y, x) * 255;
        }
      }
    }

    *outImage = out;
  }



  cv::Mat Wavelets::getLowHigh(const cv::Mat& in,
    const std::vector<float>& kernel0,
    const std::vector<float>& kernel1)
  {
    cv::Mat temp0 = convCols(in, kernel0);

    cv::Mat tempy0;

    for(int i = 0; i < temp0.rows; i += 2)
    {
      tempy0.push_back(temp0.row(i));
    }

    cv::Mat tempy01 = convRows(tempy0, kernel1);

    cv::Mat tempy001;

    cv::transpose(tempy01, tempy01);

    for(int i = 0; i < tempy01.rows; i+= 2)
    {
      tempy001.push_back(tempy01.row(i));
    }

    cv::transpose(tempy001, tempy001);

    double minVal;
    double maxVal;


    cv::minMaxLoc(tempy001, &minVal, &maxVal);

    tempy001 = tempy001 / maxVal;

    return tempy001;
  }



  cv::Mat Wavelets::getHighLow(const cv::Mat& in,
    const std::vector<float>& kernel0,
    const std::vector<float>& kernel1)
  {
    cv::Mat temp1 = convCols(in, kernel1);

    cv::Mat tempy1;

    for(int i = 0; i < temp1.rows; i += 2)
    {
      tempy1.push_back(temp1.row(i));
    }

    cv::Mat tempy10 = convRows(tempy1, kernel0);

    cv::Mat tempy010;

    cv::transpose(tempy10, tempy10);

    for(int i = 0; i < tempy10.rows; i+= 2)
    {
      tempy010.push_back(tempy10.row(i));
    }

    cv::transpose(tempy010, tempy010);

    double minVal;
    double maxVal;


    cv::minMaxLoc(tempy010, &minVal, &maxVal);

    tempy010 = tempy010 / maxVal;

    return tempy010;
  }



  cv::Mat Wavelets::getHighHigh(const cv::Mat& in,
    const std::vector<float>& kernel)
  {
    cv::Mat temp1 = convCols(in, kernel);

    cv::Mat tempy1;

    for(int i = 0; i < temp1.rows; i+= 2)
    {
      tempy1.push_back(temp1.row(i));
    }

    cv::Mat tempy11 = convRows(tempy1, kernel);

    cv::Mat tempy111;

    cv::transpose(tempy11, tempy11);

    for(int i = 0; i < tempy11.rows; i += 2)
    {
      tempy111.push_back(tempy11.row(i));
    }

    cv::transpose(tempy111, tempy111);

    double minVal;
    double maxVal;


    cv::minMaxLoc(tempy111, &minVal, &maxVal);

    tempy111 = tempy111 / maxVal;

    return tempy111;
  }

} // namespace pandora_vision

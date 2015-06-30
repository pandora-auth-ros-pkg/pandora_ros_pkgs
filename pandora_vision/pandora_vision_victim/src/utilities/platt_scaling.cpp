/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Authors:
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include <cmath>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "pandora_vision_victim/utilities/platt_scaling.h"

namespace pandora_vision
{
  PlattScaling::PlattScaling()
  {
    A_ = 1.0;
    B_ = 0.0;
  }

  PlattScaling::~PlattScaling()
  {
  }

  void PlattScaling::sigmoidTrain(const cv::Mat& predictionsMat,
      const cv::Mat& actualLabelsMat)
  {
    double prior1 = 0;
    double prior0 = 0;
    int numRows = actualLabelsMat.rows;
    for (int ii = 0; ii < numRows; ii++)
    {
      if (actualLabelsMat.at<double>(ii, 0) > 0)
        prior1 += 1;
      else
        prior0 += 1;
    }

    const int max_iter = 100;  // Maximal number of iterations
    const double min_step = 1e-10;  // Minimal step taken in line search
    const double sigma = 1e-12;  // For numerically strict PD of Hessian
    const double eps = 1e-5;
    const double hiTarget = (prior1 + 1.0) / (prior1 + 2.0);
    const double loTarget = 1.0 / (prior0 + 2.0);

    double* t = new double[numRows];
    double fApB, p, q, h11, h22, h21, g1, g2, det, dA, dB, gd, stepsize;
    double newA, newB, newf, d1, d2;
    int iter;

    // Initial Point and Initial Fun Value
    double A = 0.0;
    double B = std::log((prior0 + 1.0) / (prior1 + 1.0));
    double fval = 0.0;

    for (int ii = 0; ii <numRows; ii++)
    {
      if (actualLabelsMat.at<double>(ii, 0) > 0)
        t[ii] = hiTarget;
      else
        t[ii]=loTarget;
      fApB = predictionsMat.at<double>(ii, 0) * A + B;
      if (fApB >= 0.0)
        fval += t[ii] * fApB + std::log(1 + std::exp(-fApB));
      else
        fval += (t[ii] - 1) * fApB + std::log(1 + std::exp(fApB));
    }
    for (iter = 0; iter < max_iter; iter++)
    {
      // Update Gradient and Hessian (use H' = H + sigma I)
      h11 = sigma;  // numerically ensures strict PD
      h22 = sigma;
      h21 = 0.0;
      g1 = 0.0;
      g2 = 0.0;
      for (int ii = 0; ii < numRows; ii++)
      {
        fApB = predictionsMat.at<double>(ii, 0) * A + B;
        if (fApB >= 0.0)
        {
          p = std::exp(-fApB) / (1.0 + std::exp(-fApB));
          q = 1.0 / (1.0 + std::exp(-fApB));
        }
        else
        {
          p = 1.0 / (1.0 + std::exp(fApB));
          q = std::exp(fApB) / (1.0 + std::exp(fApB));
        }
        d2 = p * q;
        h11 += predictionsMat.at<double>(ii, 0) * predictionsMat.at<double>(ii, 0) * d2;
        h22 += d2;
        h21 += predictionsMat.at<double>(ii, 0) * d2;
        d1 = t[ii] - p;
        g1 += predictionsMat.at<double>(ii, 0) * d1;
        g2 += d1;
      }

      // Stopping Criteria
      if (std::fabs(g1) < eps && std::fabs(g2) < eps)
        break;

      // Finding Newton direction: -inv(H') * g
      det = h11 * h22 - h21 * h21;
      dA = -(h22 * g1 - h21 * g2) / det;
      dB = -(-h21 * g1 + h11 * g2) / det;
      gd = g1 * dA + g2 * dB;


      stepsize = 1;  // Line Search
      while (stepsize >= min_step)
      {
        newA = A + stepsize * dA;
        newB = B + stepsize * dB;

        // New function value
        newf = 0.0;
        for (int ii = 0; ii < numRows; ii++)
        {
          fApB = predictionsMat.at<double>(ii, 0) * newA + newB;
          if (fApB >= 0.0)
            newf += t[ii] * fApB + std::log(1 + std::exp(-fApB));
          else
            newf += (t[ii] - 1) * fApB + std::log(1 + std::exp(fApB));
        }
        // Check sufficient decrease
        if (newf < fval + 0.0001 * stepsize * gd)
        {
          A = newA;
          B = newB;
          fval = newf;
          break;
        }
        else
          stepsize = stepsize / 2.0;
      }

      if (stepsize < min_step)
      {
        std::cout << "Line search fails in two-class probability estimates" << std::endl;
        break;
      }
    }

    if (iter >= max_iter)
      std::cout << "Reaching maximal iterations in two-class probability estimates" << std::endl;
    free(t);
    A_ = A;
    B_ = B;
    std::cout << A_ << " " << B_ << std::endl;
  }

  float PlattScaling::sigmoidPredict(double predicted)
  {
    double probability;
    double fApB = predicted * A_ + B_;

    double expFApB = std::exp(-std::fabs(fApB));
    if (fApB >= 0.0)
    {
      probability =  expFApB / (1.0 + expFApB);
    }
    else
    {
      probability = 1.0 / (1.0 + expFApB);
    }
    return static_cast<float>(probability);
  }

  std::vector<float> PlattScaling::sigmoidPredict(const cv::Mat& predicted)
  {
    std::vector<float> probabilityVector;
    cv::Mat fApB = predicted * A_ + B_;
    cv::Mat absFApB = -cv::abs(fApB);
    cv::Mat expFApB;
    cv::exp(absFApB, expFApB);

    for (int ii = 0; ii < expFApB.rows; ii++)
    {
      double probability;
      if (fApB.at<double>(ii, 0) >= 0.0)
      {
        probability = expFApB.at<double>(ii, 0) / (1.0 + expFApB.at<double>(ii , 0));
      }
      else
      {
        probability = 1.0 / (1.0 + expFApB.at<double>(ii, 0));
      }
      probabilityVector.push_back(static_cast<float>(probability));
    }
    return probabilityVector;
  }

  void PlattScaling::load(const std::string& fileName)
  {
    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    fs["A"] >> A_;
    fs["B"] >> B_;
    fs.release();
  }

  void PlattScaling::save(const std::string& fileName)
  {
    cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
    fs << "A" << A_;
    fs << "B" << B_;
    fs.release();
  }
}  // namespace pandora_vision

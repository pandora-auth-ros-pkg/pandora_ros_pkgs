/*********************************************************************
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Saburo Okita saburo.okita@gmail.com
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *********************************************************************/


#ifndef PANDORA_VISION_OBSTACLE_BARREL_DETECTION_FAST_SYMMETRY_DETECTOR_H
#define PANDORA_VISION_OBSTACLE_BARREL_DETECTION_FAST_SYMMETRY_DETECTOR_H

#include <vector>
#include <utility>
#include <iostream>
#include <opencv2/opencv.hpp>
namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class FastSymmetryDetector
  {
    public:
      /**
        Based on algorithm from:
        1. Real Time Object Tracking using Reflectional Symmetry and Motion
        Wai Ho Li and Lindsay Kleeman
        Intelligent Robotics Research Centre Department of Electrical and Computer Systems Engineering Monash University, Clayton, Victoria 3800, Australia
        { Wai.Li, Lindsay.Kleeman } @eng.monash.edu.au

        2. Fast Global Reflectional Symmetry Detection for Robotic Grasping and Visual Tracking
        Wai Ho Li, Alan M. Zhang and Lindsay Kleeman
        Centre for Perceptive and Intelligent Machines in Complex Environments: Intelligent Robotics
        Monash University, Clayton
        Melbourne, Australia
        {Wai.Li, Alan.Zhang, Lindsay.Kleeman}@eng.monash.edu.au
       **/

      FastSymmetryDetector(const cv::Size imageSize, const cv::Size houghSize, const int rotResolution = 1);
      void vote(const cv::Mat& image, int minPairDist, int maxPairDist);
      inline void rotateEdges(const std::vector<cv::Point2f>& edges, int theta);

      cv::Mat getAccumulationMatrix(float thresh = 0.0);

      std::vector<std::pair<cv::Point, cv::Point> > getResult(int noOfPeaks, float threshold = -1.0f);
      std::pair<cv::Point, cv::Point> getLine(float rho, float theta);
      void getMaxDistance(float* maxDistance);
      void getYCoords(float* maxY, float* minY);

    private:
      std::vector<cv::Mat> rotMatrices;
      cv::Mat rotEdges;
      std::vector<float*> reRows;
      cv::Mat accum;

      cv::Size imageSize;
      cv::Point2f center;
      float diagonal;
      int rhoDivision;
      int rhoMax;
      int thetaMax;
      float maxDistance;
      float maxY;
      float minY;
  };
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
#endif  // PANDORA_VISION_OBSTACLE_BARREL_DETECTION_FAST_SYMMETRY_DETECTOR_H

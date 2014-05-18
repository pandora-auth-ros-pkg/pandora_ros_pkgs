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

#include "utils/edge_detection.h"

namespace pandora_vision
{
  /**
    @brief Applies the Canny edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applyCanny(const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::applyCanny : Inappropriate image depth.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("applyCanny", "computeEdges");
    #endif

    inImage.copyTo(*outImage);
    cv::Mat detected_edges;
    cv::Mat dst;
    int ratio = Parameters::Edge::canny_ratio;
    int kernel_size = Parameters::Edge::canny_kernel_size;
    int lowThreshold = Parameters::Edge::canny_low_threshold;

    // Reduce noise with a kernel 3x3
    cv::blur(*outImage, detected_edges, cv::Size(
        Parameters::Edge::canny_blur_noise_kernel_size,
        Parameters::Edge::canny_blur_noise_kernel_size));

    // Canny detector
    cv::Canny(detected_edges, detected_edges, lowThreshold,
      lowThreshold * ratio, kernel_size);

    // Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0);

    outImage->copyTo(dst, detected_edges);
    *outImage = dst; //??????????????

    #ifdef DEBUG_TIME
    Timer::tick("applyCanny");
    #endif
  }



  /**
    @brief Applies the Scharr edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applyScharr (const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::applyScharr : Inappropriate image depth.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("applyScharr", "computeEdges");
    #endif

    // appropriate values for scale, delta and ddepth
    int scale = 1;
    int delta = 0; // the value for the non-edges
    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);

    // Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    cv::Scharr(edges, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    cv::Scharr(edges, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::Mat grad_g;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad_g);

    *outImage = grad_g;

    #ifdef DEBUG_TIME
    Timer::tick("applyScharr");
    #endif
  }



  /**
    @brief Applies the Sobel edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applySobel (const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::applySobel : Inappropriate image depth.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("applySobel", "computeEdges");
    #endif

    // appropriate values for scale, delta and ddepth
    int scale = 1;
    int delta = 0; // the value for the non-edges
    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);

    // Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    cv::Sobel(edges, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    cv::Sobel(edges, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::Mat grad_g;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad_g);

    *outImage = grad_g;

    #ifdef DEBUG_TIME
    Timer::tick("applySobel");
    #endif
  }



  /**
    @brief Applies the Laplacian edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applyLaplacian (const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::applyLaplacian : Inappropriate image depth.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("applyLaplacian", "computeEdges");
    #endif

    // appropriate values for scale, delta and ddepth
    int scale = 1;
    int delta = 0; // the value for the non-edges
    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);

    cv::Laplacian(edges, *outImage, ddepth, 1, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(*outImage, *outImage);

    #ifdef DEBUG_TIME
    Timer::tick("applyLaplacian");
    #endif
  }



  /**
    @brief Applies contamination to the edges image. It keeps only the edges
    that are not iteratively neighbors to the image's limits
    @param[in,out] inImage [cv::Mat*] Input image in CV_8UC1 format
    @return void
   **/
  void EdgeDetection::applyEdgeContamination (cv::Mat* inImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyEdgeContamination", "denoiseEdges");
    #endif

    int rows = inImage->rows;
    int cols = inImage->cols;
    std::set<unsigned int> current, next, visited;

    for(unsigned int i = 0 ; i < rows ; i++)  // Border blacken
    {
      inImage->data[i * inImage->cols] = 0;
      inImage->data[i * inImage->cols + cols - 1] = 0;
    }

    for(unsigned int j = 0 ; j < cols ; j++)  // Border blacken
    {
      inImage->data[j] = 0;
      inImage->data[(rows - 1) * inImage->cols + j] = 0;
    }

    for(unsigned int i = 1 ; i < rows - 1 ; i++)  // Find outer white borders
    {
      if(inImage->data[i * inImage->cols + 1] > 0)
      {
        current.insert(i * cols + 1);
        inImage->data[i * inImage->cols + 1] = 0;
      }
      if(inImage->data[i * inImage->cols + cols - 2] > 0)
      {
        current.insert(i * cols + cols - 2);
        inImage->data[i * inImage->cols + cols - 2] = 0;
      }
    }

    for(unsigned int j = 1 ; j < cols - 1 ; j++)  // Find outer white borders
    {
      if(inImage->data[1 * inImage->cols + j] > 0)
      {
        current.insert(1 * cols + j);
        inImage->data[1 * inImage->cols + j] = 0;
      }

      if(inImage->data[(rows - 2) * inImage->cols + j] > 0)
      {
        current.insert((rows - 2) * cols + j);
        inImage->data[(rows - 2) * inImage->cols + j] = 0;
      }
    }

    while(current.size() != 0)  // Iterative contamination
    {
      for(std::set<unsigned int>::iterator i = current.begin() ;
        i != current.end() ; i++)
      {
        int x = *i / cols;
        int y = *i % cols;

        if(inImage->data[(x - 1) * inImage->cols + y - 1] != 0)
        {
          next.insert((x - 1) * cols + y - 1);
          inImage->data[(x - 1) * inImage->cols + y - 1] = 0;
        }

        if(inImage->data[(x - 1) * inImage->cols + y] != 0)
        {
          next.insert((x - 1) * cols + y);
          inImage->data[(x - 1) * inImage->cols + y] = 0;
        }

        if(inImage->data[(x - 1) * inImage->cols + y + 1] != 0)
        {
          next.insert((x - 1) * cols + y + 1);
          inImage->data[(x - 1) * inImage->cols + y + 1] = 0;
        }

        if(inImage->data[x * inImage->cols + y + 1] != 0)
        {
          next.insert(x * cols + y + 1);
          inImage->data[x * inImage->cols + y + 1] = 0;
        }

        if(inImage->data[x * inImage->cols + y - 1] != 0)
        {
          next.insert(x * cols + y - 1);
          inImage->data[x * inImage->cols + y - 1] = 0;
        }

        if(inImage->data[(x + 1) * inImage->cols + y - 1] != 0)
        {
          next.insert((x + 1) * cols + y - 1);
          inImage->data[(x + 1) * inImage->cols + y - 1] = 0;
        }

        if(inImage->data[(x + 1) * inImage->cols + y] != 0)
        {
          next.insert((x + 1) * cols + y);
          inImage->data[(x + 1) * inImage->cols + y] = 0;
        }

        if(inImage->data[(x + 1) * inImage->cols + y + 1] != 0)
        {
          next.insert((x + 1) * cols + y + 1);
          inImage->data[(x + 1) * inImage->cols + y + 1] = 0;
        }
      }
      current.swap(next);
      next.clear();
    }

    #ifdef DEBUG_TIME
    Timer::tick("applyEdgeContamination");
    #endif
  }



  /**
    @brief Apply the anisotropic diffusion technique as to facilitate the
    edge detection of low contrast regions.
    @param[in] inImage [const cv::Mat &] The input image in CV_32FC1 format
    @param[out] outImage [cv::Mat*] The output image in CV_32FC1 format
    @param[in] iterations [const int&] The number of iterations to execute.
    @param[in] method [const int&] Diffusion equation 0 or 1. see below for more
    @documentation http://www.csse.uwa.edu.au/~pk/research/matlabfns/
    @return void
   **/
  void EdgeDetection::anisotropicDiffusion (const cv::Mat& inImage,
    cv::Mat* outImage, const int& iterations, const int& method)
  {
    #ifdef DEBUG_TIME
    Timer::start("anisotropicDiffusion");
    #endif

    int kappa = 1;
    float lamda = 0.25;

    inImage.copyTo(*outImage);

    cv::Mat deltaNorth = cv::Mat::zeros(outImage->size(), CV_32FC1);
    cv::Mat deltaSouth = cv::Mat::zeros(outImage->size(), CV_32FC1);
    cv::Mat deltaEast  = cv::Mat::zeros(outImage->size(), CV_32FC1);
    cv::Mat deltaWest  = cv::Mat::zeros(outImage->size(), CV_32FC1);

    cv::Mat cN = cv::Mat::zeros(outImage->size(), CV_32FC1);
    cv::Mat cS = cv::Mat::zeros(outImage->size(), CV_32FC1);
    cv::Mat cE = cv::Mat::zeros(outImage->size(), CV_32FC1);
    cv::Mat cW = cv::Mat::zeros(outImage->size(), CV_32FC1);

    for (unsigned int iter = 0; iter < iterations; iter++)
    {
      for (unsigned int rows = 1; rows < outImage->rows - 1; rows++)
      {
        for (unsigned int cols = 1; cols < outImage->cols - 1; cols ++)
        {
          deltaNorth.at<float>(rows, cols) =
            outImage->at<float>(rows, cols - 1) -
            outImage->at<float>(rows, cols);

          deltaSouth.at<float>(rows, cols) =
            outImage->at<float>(rows, cols + 1) -
            outImage->at<float>(rows, cols);

          deltaEast.at<float>(rows, cols) =
            outImage->at<float>(rows + 1, cols) -
            outImage->at<float>(rows, cols);

          deltaWest.at<float>(rows, cols) =
            outImage->at<float>(rows - 1, cols) -
            outImage->at<float>(rows, cols);
        }
      }

      if (method == 0)
      {
        for (unsigned int rows = 0; rows < outImage->rows; rows++)
        {
          for (unsigned int cols = 0; cols < outImage->cols; cols ++)
          {
            cN.at<float>(rows, cols) =
              exp(-pow((deltaNorth.at<float>(rows, cols) / kappa), 2));
            cS.at<float>(rows, cols) =
              exp(-pow((deltaSouth.at<float>(rows, cols) / kappa), 2));
            cE.at<float>(rows, cols) =
              exp(-pow((deltaEast.at<float>(rows, cols) / kappa), 2));
            cW.at<float>(rows, cols) =
              exp(-pow((deltaWest.at<float>(rows, cols) / kappa), 2));

          }
        }
      }
      else if (method == 1)
      {
        for (unsigned int rows = 0; rows < outImage->rows; rows++)
        {
          for (unsigned int cols = 0; cols < outImage->cols; cols ++)
          {
            cN.at<float>(rows, cols) =
              1 / (1 + pow((deltaNorth.at<float>(rows, cols) / kappa), 2));
            cS.at<float>(rows, cols) =
              1 / (1 + pow((deltaSouth.at<float>(rows, cols) / kappa), 2));
            cE.at<float>(rows, cols) =
              1 / (1 + pow((deltaEast.at<float>(rows, cols) / kappa), 2));
            cW.at<float>(rows, cols) =
              1 / (1 + pow((deltaWest.at<float>(rows, cols) / kappa), 2));
          }
        }
      }
      else if (method == 2)
      {
        for (unsigned int rows = 0; rows < outImage->rows; rows++)
        {
          for (unsigned int cols = 0; cols < outImage->cols; cols ++)
          {
            cN.at<float>(rows, cols) =
              exp(pow((deltaNorth.at<float>(rows, cols) * kappa), 2));
            cS.at<float>(rows, cols) =
              exp(pow((deltaSouth.at<float>(rows, cols) * kappa), 2));
            cE.at<float>(rows, cols) =
              exp(pow((deltaEast.at<float>(rows, cols) * kappa), 2));
            cW.at<float>(rows, cols) =
              exp(+pow((deltaWest.at<float>(rows, cols) * kappa), 2));

          }
        }
      }

      *outImage += lamda * (cN.mul(deltaNorth) + cS.mul(deltaSouth)
        + cE.mul(deltaEast) + cW.mul(deltaWest));
    }
    #ifdef DEBUG_TIME
    Timer::tick("anisotropicDiffusion");
    #endif
  }



  /**
    @brief Takes as input a depth image containing floats,
    locates the edges in it and tries to clear as much noise as possible
    in the edges image. As noise we identify everything that is not,
    or does not look like, hole-like shapes,
    with the knowledge that these shapes might be open curves, or that
    holes-like shapes in a edge image are not connected to anything else,
    ergo they are standalone shapes in it.
    It outputs a binary image that contains areas that we wish to validate
    as holes.
    @param[in] inImage [const cv::Mat&] The depth image extracted from the
    depth sensor, of type CV_32FC1
    @param[out] edges [cv::Mat*] The final denoised edges image that
    corresponds to the input image
    @return void
   **/
  void EdgeDetection::computeDepthEdges(const cv::Mat& inImage, cv::Mat* edges)
  {
    if (inImage.type() != CV_32FC1)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::computeEdges : Inappropriate image type.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("computeEdges", "findHoles");
    #endif


    cv::Mat denoisedDepthImageEdges;
    cv::Mat visualizableDenoisedImage;
    cv::Mat tempImg;

    inImage.copyTo(tempImg);

    // Facilitate the edge detection by converting the 32FC1 image \
    values to a range of 0-255
      visualizableDenoisedImage = Visualization::scaleImageForVisualization
      (tempImg, Parameters::Image::scale_method);


    // from now onwards every image is in the range of 0-255
    if (Parameters::Edge::edge_detection_method == 0)
    {
      applyCanny(visualizableDenoisedImage, &denoisedDepthImageEdges);
    }
    else if (Parameters::Edge::edge_detection_method == 1)
    {
      applyScharr(visualizableDenoisedImage, &denoisedDepthImageEdges);
    }
    else if (Parameters::Edge::edge_detection_method == 2)
    {
      applySobel(visualizableDenoisedImage, &denoisedDepthImageEdges);
    }
    else if (Parameters::Edge::edge_detection_method == 3)
    {
      applyLaplacian(visualizableDenoisedImage, &denoisedDepthImageEdges);
    }
    else if (Parameters::Edge::edge_detection_method == 4) // Mixed mode
    {
      if (Parameters::Edge::mixed_edges_toggle_switch == 1)
      {
        applyScharr(visualizableDenoisedImage, &denoisedDepthImageEdges);
        Parameters::Edge::mixed_edges_toggle_switch = 2;
      }
      else if (Parameters::Edge::mixed_edges_toggle_switch == 2)
      {
        applySobel(visualizableDenoisedImage, &denoisedDepthImageEdges);
        Parameters::Edge::mixed_edges_toggle_switch = 1;
      }
    }


    cv::threshold(denoisedDepthImageEdges, denoisedDepthImageEdges,
      Parameters::Edge::denoised_edges_threshold, 255, 3);

    // Make all non zero pixels have a value of 255
    cv::threshold(denoisedDepthImageEdges, denoisedDepthImageEdges, 0, 255, 0);

    // Denoise the edges found
    denoisedDepthImageEdges.copyTo(*edges);
    denoiseEdges(edges);

    #ifdef DEBUG_TIME
    Timer::tick("computeEdges");
    #endif
  }



  /**
    @brief Takes as input a RGB image of type CV_8UC3,
    locates the edges in it and tries to clear as much noise as possible
    in the edges image. As noise we identify everything that is not,
    or does not look like, hole-like shapes,
    with the knowledge that these shapes might be open curves, or that
    holes-like shapes in a edge image are not connected to anything else,
    ergo they are standalone shapes in it.
    It outputs a binary image that contains areas that we wish to validate
    as holes.
    @param[in] inImage [const cv::Mat&] The RGB image of type CV_32FC1
    @param[in] extractionMethod [const int&] Chooses by which process the
    edges will be extracted. 0 for extraction via segmentation,
    1 for extraction via backprojection and watersheding
    @param[in] segmentationMethod [const int&] The method by which the
    segmentation of @param inImage will be performed.
    @param[in] inHistogram [const cv::MatND&] The model histogram needed
    in order to obtain the backprojection of @param inImage
    @param[out] edges [cv::Mat*] The final denoised edges image that
    corresponds to the input image
    @return void
   **/
  void EdgeDetection::computeRgbEdges(const cv::Mat& inImage,
    const int& extractionMethod, const int& segmentationMethod,
    const cv::MatND& inHistogram, cv::Mat* edges)
  {
    if (inImage.type() != CV_8UC3)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::computeRgbEdges: Inappropriate image type.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("computeRgbEdges", "findHoles");
    #endif

    if (extractionMethod == 0)
    {
      produceEdgesViaSegmentation(inImage, segmentationMethod, edges);
    }
    else if (extractionMethod == 1)
    {
      produceEdgesViaBackprojection(inImage, inHistogram, edges);
    }

    // Apply a threshold so as to get rid of, what we perceive to be, noise
    cv::threshold(*edges, *edges,
      Parameters::Edge::denoised_edges_threshold, 255, 3);

    // Make all non zero pixels have a value of 255
    cv::threshold(*edges, *edges, 0, 255, 0);

    // Denoise the edges image
    EdgeDetection::denoiseEdges(edges);

    #ifdef DEBUG_TIME
    Timer::tick("computeRgbEdges");
    #endif
  }



  /**
    @brief Connects each point of a number of pair of points  with a line or an
    elliptic arc
    @param[in,out] inImage [cv::Mat*] The image whose selected points will be
    connected and line or the elliptic arc drawn on
    @param[in] pairs [const std::vector<std::pair<GraphNode,GraphNode> >&]
    The vector of pair points
    @param[in] method [const int&] Denotes the connection type. 0 for line,
    1 for elliptic arc
    @return void
   **/
  void EdgeDetection::connectPairs(cv::Mat* inImage,
    const std::vector<std::pair<GraphNode, GraphNode> >& pairs,
    const int& method)
  {
    if (inImage->depth() != CV_8U)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::connectPairs: Inappropriate image depth.");
      #endif

      return;
    }

    #ifdef DEBUG_SHOW
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    if(Parameters::Debug::show_connect_pairs) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Connection before";
      msgs.push_back(msg);
      cv::Mat tmp;
      inImage->copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::start("connectPairs", "denoiseEdges");
    #endif

    if (method == 0)
    {
      for (unsigned int i = 0; i < pairs.size(); i++)
      {
        cv::line(
          *inImage,
          cv::Point(pairs[i].first.y, pairs[i].first.x),
          cv::Point(pairs[i].second.y, pairs[i].second.x),
          cv::Scalar(255, 255, 255), 1, 8);
      }
    }
    else if (method == 1)
    {

      cv::Mat inImageDrawnOnce;
      cv::Mat addedArcs =  cv::Mat(inImage->rows, inImage->cols, CV_8UC1,
        cv::Scalar::all(0));

      for (unsigned int i = 0; i < pairs.size(); i++)
      {
        inImage->copyTo(inImageDrawnOnce);

        float pairsDistance = sqrt(
          pow((pairs[i].first.x - pairs[i].second.x), 2) +
          pow((pairs[i].first.y - pairs[i].second.y), 2));

        cv::Point2f bisectorPoint(
          round((pairs[i].first.y + pairs[i].second.y) / 2),
          round((pairs[i].first.x + pairs[i].second.x) / 2));

        float bisectorAngle = atan2(
          pairs[i].second.x - pairs[i].first.x,
          pairs[i].first.y - pairs[i].second.y);

        // Given the line that is perpendicular to the line that connects
        // the pair points, move on it one point at a time, in opposite
        // directions. The first non-zero point found will be one of the
        // curve that the pair lies on.
        bool foundOutlinePoint = false;
        cv::Point2f outlinePoint;
        int counter = 0;
        bool inLimitsOne = true;
        bool inLimitsTwo = true;

        while(!foundOutlinePoint)
        {
          counter++;

          if (bisectorPoint.y + counter * cos(bisectorAngle) > inImage->rows - 1
            || bisectorPoint.y + counter * cos(bisectorAngle) < 0 ||
            bisectorPoint.x + counter * sin(bisectorAngle) > inImage->cols - 1
            || bisectorPoint.x + counter * sin(bisectorAngle) < 0)
          {
            inLimitsOne = false;
          }

          if (bisectorPoint.y - counter * cos(bisectorAngle) > inImage->rows - 1
            || bisectorPoint.y - counter * cos(bisectorAngle) < 0 ||
            bisectorPoint.x - counter * sin(bisectorAngle) > inImage->cols - 1
            || bisectorPoint.x - counter * sin(bisectorAngle) < 0)
          {
            inLimitsTwo= false;
          }

          if (inLimitsOne)
          {
            for (int m = -1; m < 2; m++)
            {
              for (int n = -1; n < 2; n++)
              {
                if (inImage->at<uchar>(
                    bisectorPoint.y + m + counter * cos(bisectorAngle),
                    bisectorPoint.x + n + counter * sin(bisectorAngle)) != 0)
                {
                  outlinePoint = cv::Point2f(
                    bisectorPoint.x + n + counter * sin(bisectorAngle),
                    bisectorPoint.y + m + counter * cos(bisectorAngle));

                  foundOutlinePoint = true;
                }
              }
            }
          }

          if (inLimitsTwo)
          {
            for (int m = -1; m < 2; m++)
            {
              for (int n = -1; n < 2; n++)
              {
                if (inImage->at<uchar>(
                    bisectorPoint.y + m - counter * cos(bisectorAngle),
                    bisectorPoint.x + n - counter * sin(bisectorAngle)) != 0)
                {
                  outlinePoint = cv::Point2f(
                    bisectorPoint.x + n - counter * sin(bisectorAngle),
                    bisectorPoint.y + m - counter * cos(bisectorAngle));
                  foundOutlinePoint = true;
                }
              }
            }
          }
        }

        if (foundOutlinePoint)
        {
          float outlineBisectorPointDist = sqrt(
            pow(bisectorPoint.x - outlinePoint.x, 2) +
            pow(bisectorPoint.y - outlinePoint.y, 2));


          // If the curve is close to a straight line,
          // do not connect pair[i].first and pair[i].second
          if (pairsDistance >=
            Parameters::Outline::AB_to_MO_ratio * outlineBisectorPointDist)
          {
            continue;
          }

          float majorAxis = pairsDistance / 2;

          float minorAxis = outlineBisectorPointDist / 2 < pairsDistance / 4 ?
            outlineBisectorPointDist / 2 : pairsDistance / 4;

          float pairFirstToOutlinePointAngle = atan2(
            outlinePoint.y - pairs[i].first.x,
            outlinePoint.x - pairs[i].first.y);

          float pairSecondToOutlinePointAngle = atan2(
            outlinePoint.y - pairs[i].second.x,
            outlinePoint.x - pairs[i].second.y);

          // map the angles to the standard polar system
          if (pairFirstToOutlinePointAngle > 0)
          {
            pairFirstToOutlinePointAngle = 3.1415926535897
              - std::abs(pairFirstToOutlinePointAngle);
          }
          else
          {
            pairFirstToOutlinePointAngle = -3.1415926535897
              + std::abs(pairFirstToOutlinePointAngle);
          }

          if (pairSecondToOutlinePointAngle > 0)
          {
            pairSecondToOutlinePointAngle = 3.1415926535897
              - std::abs(pairSecondToOutlinePointAngle);
          }
          else
          {
            pairSecondToOutlinePointAngle = -3.1415926535897
              + std::abs(pairSecondToOutlinePointAngle);
          }

          float pairsAngle;
          float defaultAngle = atan2(
            pairs[i].first.x - pairs[i].second.x,
            pairs[i].first.y - pairs[i].second.y);

          // Both pair points are above the X axis
          if (pairFirstToOutlinePointAngle > 0 &&
            pairSecondToOutlinePointAngle > 0)
          {
            if (pairFirstToOutlinePointAngle < pairSecondToOutlinePointAngle)
            {
              pairsAngle = defaultAngle + 3.1415926535897;
            }
            else
            {
              pairsAngle = defaultAngle;
            }
          }

          // Both pair points are below the X axis
          if (pairFirstToOutlinePointAngle < 0 &&
            pairSecondToOutlinePointAngle < 0)
          {
            if (pairFirstToOutlinePointAngle > pairSecondToOutlinePointAngle)
            {
              pairsAngle = defaultAngle;
            }
            else
            {
              pairsAngle = defaultAngle + 3.1415926535897;
            }
          }

          // Pair point 2 is above the X axis while pair point 1 below it
          if (pairFirstToOutlinePointAngle < 0 &&
            pairSecondToOutlinePointAngle > 0)
          {
            if (-pairFirstToOutlinePointAngle
              + pairSecondToOutlinePointAngle < 3.1415926535897)
            {
              pairsAngle = defaultAngle + 3.1415926535897;
            }
            else
            {
              pairsAngle = defaultAngle;
            }
          }

          // Pair point 1 is above the X axis while pair point 2 below it
          if (pairFirstToOutlinePointAngle > 0 &&
            pairSecondToOutlinePointAngle < 0)
          {
            if (pairFirstToOutlinePointAngle
              - pairSecondToOutlinePointAngle < 3.1415926535897)
            {
              pairsAngle = defaultAngle;
            }
            else
            {
              pairsAngle = defaultAngle + 3.1415926535897;
            }
          }

          // size arg 1: length of the major axis. always pairsDistance / 2
          // size arg 2: length of the minor axis.
          cv::ellipse(
            inImageDrawnOnce,
            bisectorPoint,
            cv::Size(majorAxis, minorAxis),
            pairsAngle * 180 / 3.1415926535897,
            0,
            180,
            cv::Scalar(255, 255, 255));

          for (unsigned int rows = 0; rows < inImage->rows; rows++)
          {
            for (unsigned int cols = 0; cols < inImage->cols; cols++)
            {
              if (inImageDrawnOnce.at<unsigned char>(rows, cols) != 0 &&
                inImage->at<unsigned char>(rows, cols) == 0)
              {
                addedArcs.at<uchar>(rows, cols) = 255;
              }
            }
          }
        }
      }

      *inImage += addedArcs;
    }

    #ifdef DEBUG_TIME
    Timer::tick("connectPairs");
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_connect_pairs) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Connection after";
      msgs.push_back(msg);
      cv::Mat tmp;
      inImage->copyTo(tmp);
      imgs.push_back(tmp);
    }
    if(Parameters::Debug::show_connect_pairs) // Debug
    {
      Visualization::multipleShow("connectPairs function", imgs, msgs, 1200, 1);
    }
    #endif
  }



  /**
   * @brief Enhances an image's contrast, regardless of its format.
   * @param[in] inImage [const cv::Mat &] The input image
   * @param[out] outImage [cv::Mat*] The contrast-enhanced image
   * @return void
   **/
  void EdgeDetection::enhanceContrast(const cv::Mat& inImage, cv::Mat* outImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("enhanceContrast");
    #endif

    inImage.convertTo(*outImage, -1, Parameters::Edge::contrast_enhance_alpha,
      Parameters::Edge::contrast_enhance_beta);

    #ifdef DEBUG_TIME
    Timer::tick("enhanceContrast");
    #endif
  }


  /**
    @brief Takes an input image in unsigned char format and tries to isolate
    hole-like shapes so as to facilitate the blob detection process
    @param[in,out] img [cv::Mat*] The input image in unsigned char format
    @return void
   **/
  void EdgeDetection::denoiseEdges(cv::Mat* img)
  {
    if (img->depth() != CV_8U)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::denoiseEdges: Inappropriate image depth.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("denoiseEdges", "computeEdges");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("Sector #1", "denoiseEdges");
    #endif

    #ifdef DEBUG_SHOW
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial Edges";
      msgs.push_back(msg);
      cv::Mat tmp;
      img->copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    cv::Mat contaminatedEdges;
    img->copyTo(contaminatedEdges);

    // Perform edge contamination:
    // remove all non-zero pixels adjacent to the image's borders
    applyEdgeContamination(&contaminatedEdges);

    #ifdef DEBUG_TIME
    Timer::tick("Sector #1");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("Sector #2", "denoiseEdges");
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After edge contamination";
      msgs.push_back(msg);
      cv::Mat tmp;
      contaminatedEdges.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // Eliminate closed shapes
    for(unsigned int i = 0 ; i < img->rows ; i++)
    {
      contaminatedEdges.at<unsigned char>(i, 0) = 0;
      contaminatedEdges.at<unsigned char>(i, img->cols - 1) = 0;
    }
    for(unsigned int i = 0 ; i < img->cols ; i++)
    {
      contaminatedEdges.at<unsigned char>(0, i) = 0;
      contaminatedEdges.at<unsigned char>(img->rows - 1, i) = 0;
    }

    // Apply thinning to the contaminated original edges
    cv::Mat thinnedImage;
    contaminatedEdges.copyTo(thinnedImage);

    Morphology::thinning(contaminatedEdges, &thinnedImage, 100);

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After thinning";
      msgs.push_back(msg);
      cv::Mat tmp;
      thinnedImage.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // By pruning the thinned image,
    // all pixels that are open-ended are eliminated,
    // thus leaving behind only those shapes whose ends are connected, aka the
    // closed shapes
    cv::Mat thinnedClosedLines;
    thinnedImage.copyTo(thinnedClosedLines);
    Morphology::pruningStrictIterative(&thinnedClosedLines, 1000);

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After pruning";
      msgs.push_back(msg);
      cv::Mat tmp;
      thinnedClosedLines.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // The thinnedOpenLines image now features only the thinned,
    // open-ended shapes
    cv::Mat thinnedOpenLines = thinnedImage - thinnedClosedLines;

    // We want to preserve the original (unthinned) outline of the closed shapes
    // and connect the ends of open-ended shapes which are thinned to this point.
    // So, first, we need to identify the closed shapes in the edge contaminated
    // image. These will be stored in the closedLines image.
    cv::Mat closedLines;
    thinnedClosedLines.copyTo(closedLines);

    bool finished = false;

    while(!finished)
    {
      cv::Mat marker;
      closedLines.copyTo(marker);

      for (int rows = 1; rows < img->rows - 1; rows++)
      {
        for (int cols = 1; cols < img->cols - 1; cols++)
        {
          if (closedLines.at<unsigned char>(rows, cols) == 0
            && contaminatedEdges.at<unsigned char>(rows, cols) == 255)
          {
            for (int r = -1; r < 2; r++)
            {
              for (int c = -1; c < 2; c++)
              {
                if (contaminatedEdges.at<unsigned char>(rows + r, cols + c) != 0
                  && closedLines.at<unsigned char>(rows + r, cols + c) != 0)
                {
                  closedLines.at<unsigned char>(rows, cols) = 255;
                }
              }
            }
          }
        }
      }

      // The operation has finished when all the original closed shapes have
      // been found
      if (cv::countNonZero(closedLines - marker) == 0)
      {
        finished = true;
      }
    }

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : The origninal closed shapes";
      msgs.push_back(msg);
      cv::Mat tmp;
      closedLines.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Without closed shapes";
      msgs.push_back(msg);
      cv::Mat tmp;
      thinnedOpenLines.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("Sector #2");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("Sector #3");
    #endif

    // In the image that features only open-ended shapes, find their end points.
    // We will then check if they are eligible for connection, and if they are,
    // these points will be connected with each other

    std::vector<std::set<unsigned int> > lines;
    std::vector<std::pair<GraphNode, GraphNode> > farPts;
    bool hasFinished = false;

    while(!hasFinished)
    {
      hasFinished = true;
      for(unsigned int i = 1 ; i < thinnedOpenLines.rows - 1; i++)
      {
        for(unsigned int j = 1 ; j < thinnedOpenLines.cols - 1; j++)
        {
          if(thinnedOpenLines.at<unsigned char> (i, j) != 0)
          {
            std::set<unsigned int> ret;
            std::pair<GraphNode, GraphNode> pts =
              findNeighs(&thinnedOpenLines, i, j, &ret);

            if(ret.size() > Parameters::Outline::minimum_curve_points)
            {
              lines.push_back(ret);
              farPts.push_back(pts);
            }

            hasFinished = false;
            break;
          }
        }

        if(!hasFinished)
        {
          break;
        }
      }
    }

    for(unsigned int i = 0 ; i < lines.size() ; i++)
    {
      for(std::set<unsigned int>::iterator it = lines[i].begin() ;
        it != lines[i].end() ; it++)
      {
        thinnedOpenLines.data[*it] = 255;
      }
    }

    // Connect the end points of open shapes
    connectPairs(&thinnedOpenLines, farPts, 1);

    // Since what could be connected was connected, the rest of the open-ended
    // shapes are not needed.
    Morphology::pruningStrictIterative(&thinnedOpenLines, 1000);

    #ifdef DEBUG_TIME
    Timer::tick("Sector #3");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("Sector #4");
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After connection of distant edges";
      msgs.push_back(msg);
      cv::Mat tmp;
      thinnedOpenLines.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // Re-enable the closed shapes
    *img = thinnedOpenLines + closedLines;

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After re-insertion of closed shapes";
      msgs.push_back(msg);
      cv::Mat tmp;
      img->copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    // Extract only the outer border of closed shapes
    getShapesClearBorderSimple(img);

    #ifdef DEBUG_TIME
    Timer::tick("Sector #4");
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After clear borders";
      msgs.push_back(msg);
      cv::Mat tmp;
      img->copyTo(tmp);
      imgs.push_back(tmp);
    }
    if(Parameters::Debug::show_denoise_edges) // Debug
    {
      Visualization::multipleShow("denoiseEdges function", imgs, msgs,
        Parameters::Debug::show_denoise_edges_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("denoiseEdges");
    #endif
  }


  /**
    @brief Identifies in which curve a point lies on and returns the
    curve's two end points. If a point does not lie on a curve,
    the pair returned has both coordinates (x,y) set to zero, and the
    size of @param ret is one.
    @param[in] img [cv::Mat*] The input binary image
    @param[in] x_ [const int&] The x coordinate of the point
    @param[in] y_ [const int&] The y coordinate of the point
    @param[out] ret [std::set<unsigned int>&] The points that represent the
    curve on which the point lies on
    @return edgePoints [std::pair<GraphNode, GraphNode>*] The curve's pair of
    end points
   **/
  std::pair<GraphNode, GraphNode> EdgeDetection::findNeighs(
    cv::Mat* img, const int& x_, const int& y_, std::set<unsigned int>* ret)
  {
    #ifdef DEBUG_TIME
    Timer::start("findNeighs", "denoiseEdges");
    #endif

    std::vector<unsigned int> current, next;
    std::set<unsigned int> currs;
    std::vector<GraphNode*> nodes;
    std::vector<GraphNode*> originCurr, originNext;
    GraphNode* father = new GraphNode(x_, y_);

    ret->insert(x_ * img->cols + y_);
    current.push_back(x_ * img->cols + y_);
    originCurr.push_back(father);
    nodes.clear();
    nodes.push_back(father);
    father->dist = 0;
    unsigned int stepCounter = 0;
    while(current.size() != 0)
    {
      next.clear();
      originNext.clear();
      for(unsigned int i = 0 ; i < current.size() ; i++)
      {
        std::vector<unsigned int> currNext;
        int x = current[i] / img->cols;
        int y = current[i] % img->cols;
        img->at<unsigned char>(x, y) = 0;
        unsigned int counter = 0;
        if(img->at<unsigned char>(x - 1, y - 1) != 0 &&
          currs.find((x-1) * img->cols + y - 1) == currs.end() &&
          ret->find((x-1) * img->cols + y - 1) == ret->end())
        {
          next.push_back((x-1) * img->cols + y - 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x-1) * img->cols + y - 1);
          counter++;
        }
        if(img->at<unsigned char>(x - 1, y) != 0 &&
          currs.find((x-1) * img->cols + y) == currs.end() &&
          ret->find((x-1) * img->cols + y) == ret->end())
        {
          next.push_back((x-1) * img->cols + y);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x-1) * img->cols + y);
          counter++;
        }
        if(img->at<unsigned char>(x - 1, y + 1) != 0 &&
          currs.find((x-1) * img->cols + y + 1) == currs.end() &&
          ret->find((x-1) * img->cols + y + 1) == ret->end())
        {
          next.push_back((x-1) * img->cols + y + 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x-1) * img->cols + y + 1);
          counter++;
        }

        if(img->at<unsigned char>(x, y - 1) != 0 &&
          currs.find((x) * img->cols + y - 1) == currs.end() &&
          ret->find((x) * img->cols + y - 1) == ret->end())
        {
          next.push_back((x) * img->cols + y - 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x) * img->cols + y - 1);
          counter++;
        }
        if(img->at<unsigned char>(x, y + 1) != 0 &&
          currs.find((x) * img->cols + y + 1) == currs.end() &&
          ret->find((x) * img->cols + y + 1) == ret->end())
        {
          next.push_back((x) * img->cols + y + 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x) * img->cols + y + 1);
          counter++;
        }

        if(img->at<unsigned char>(x + 1, y - 1) != 0 &&
          currs.find((x+1) * img->cols + y - 1) == currs.end() &&
          ret->find((x+1) * img->cols + y - 1) == ret->end())
        {
          next.push_back((x+1) * img->cols + y - 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x+1) * img->cols + y - 1);
          counter++;
        }
        if(img->at<unsigned char>(x + 1, y) != 0 &&
          currs.find((x+1) * img->cols + y) == currs.end() &&
          ret->find((x+1) * img->cols + y) == ret->end())
        {
          next.push_back((x+1) * img->cols + y);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x+1) * img->cols + y);
          counter++;
        }
        if(img->at<unsigned char>(x + 1, y + 1) != 0 &&
          currs.find((x+1) * img->cols + y + 1) == currs.end() &&
          ret->find((x+1) * img->cols + y + 1) == ret->end())
        {
          next.push_back((x+1) * img->cols + y + 1);
          originNext.push_back(originCurr[i]);
          currNext.push_back(originNext.size() - 1);
          ret->insert((x+1) * img->cols + y + 1);
          counter++;
        }

        if(counter == 0)  // This is edge
        {
          GraphNode* temp = new GraphNode(x, y);
          nodes.push_back(temp);

          originCurr[i]->connections.push_back(temp);
          temp->connections.push_back(originCurr[i]);

          temp->dist = stepCounter - originCurr[i]->dist;

          originCurr[i]->dists.push_back(temp->dist);
          temp->dists.push_back(temp->dist);
        }
        else if(counter >= 2) // This is joint
        {
          GraphNode* temp = new GraphNode(x, y);
          nodes.push_back(temp);
          originCurr[i]->connections.push_back(temp);
          temp->connections.push_back(originCurr[i]);
          temp->dist = stepCounter - originCurr[i]->dist;
          originCurr[i]->dists.push_back(temp->dist);
          temp->dists.push_back(temp->dist);
          for(unsigned int j = 0 ; j < currNext.size() ; j++)
          {
            originNext[currNext[j]] = temp;
          }
        }

        currNext.clear();
      }
      stepCounter++;
      current.swap(next);
      currs.clear();
      for(unsigned int k = 0 ; k < current.size() ; k++)
      {
        currs.insert(current[k]);
      }
      originCurr.swap(originNext);
    }

    std::pair<GraphNode, GraphNode> edgePoints;
    // If it is small avoid the fuzz
    if(ret->size() < Parameters::Outline::minimum_curve_points)
    {
      return edgePoints;
    }

    // Find larger dist between nodes
    int maxDist = 0;

    for(unsigned int i = 0 ; i < nodes.size(); i++)
    {
      std::vector<GraphNode*> current, next;
      std::map<GraphNode*, int> dists;
      std::set<GraphNode*> visited;

      current.push_back(nodes[i]);
      visited.insert(nodes[i]);
      for(unsigned int kk = 0 ; kk < nodes.size() ; kk++)
      {
        dists.insert(std::pair<GraphNode*, int>(nodes[i], 0));
      }
      while(current.size() != 0)
      {
        next.clear();
        for(unsigned int c = 0 ; c < current.size() ; c++)
        {
          for(unsigned int n = 0 ; n < current[c]->connections.size(); n++)
          {
            if(visited.find(current[c]->connections[n]) == visited.end())
            {
              visited.insert(current[c]->connections[n]);
              dists[current[c]->connections[n]] +=
                dists[current[c]] + current[c]->dists[n];
              next.push_back(current[c]->connections[n]);
            }
          }
        }
        current.swap(next);
      }
      for(unsigned int d = 0 ; d < nodes.size(); d++)
      {
        if(dists[nodes[d]] > maxDist)
        {
          maxDist = dists[nodes[d]];
          edgePoints.first.x = nodes[i]->x;
          edgePoints.first.y = nodes[i]->y;
          edgePoints.second.x = nodes[d]->x;
          edgePoints.second.y = nodes[d]->y;
        }
      }
    }

    for(unsigned int d = 0 ; d < nodes.size(); d++)
    {
      delete nodes[d];
    }

    #ifdef DEBUG_TIME
    Timer::tick("findNeighs");
    #endif

    return edgePoints;
  }



  /**
    @brief Fills an image with random colours per image segment
    @param[in,out] image [cv::Mat*] The image to be processed
    (see http://docs.opencv.org/modules/imgproc/doc/
    miscellaneous_transformations.html#floodfill)
    @return void
   **/
  void EdgeDetection::floodFillPostprocess(cv::Mat* image)
  {
    if (image->type() != CV_8UC3)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::floodFillPostprocess Inappropriate image type.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("floodFillPostprocess", "produceEdgesViaSegmentation");
    #endif

    cv::RNG rng = cv::theRNG();
    cv::Mat mask = cv::Mat::zeros(image->rows + 2, image->cols + 2, CV_8UC1);

    // Get a pointer on mask to speed-up execution
    unsigned char* mask_ptr = mask.ptr();

    for (int rows = 0; rows < image->rows; rows++)
    {
      for (int cols = 0; cols < image->cols; cols++)
      {
        if (mask_ptr[(rows + 1) * mask.cols + (cols + 1)] == 0)
        {
          cv::Scalar newVal(rng(256), rng(256), rng(256));

          // Fill this segment with a random colour
          cv::floodFill(*image, mask, cv::Point(cols, rows), newVal, 0,
            cv::Scalar::all(Parameters::Rgb::floodfill_lower_colour_difference),
            cv::Scalar::all(Parameters::Rgb::floodfill_upper_colour_difference)
            );
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("floodFillPostprocess");
    #endif
  }



  /**
    @brief With an binary input image (quantized in 0 and 255 levels),
    this function fills closed regions, at first, and then extracts the outline
    of each region. Used when there is a closed region with garbage pixels with
    a value of 255 within it.
    @param[in,out] inImage [cv::Mat*] The input image
    @return void
   **/
  void EdgeDetection::getShapesClearBorder (cv::Mat* inImage)
  {
    if (inImage->type() != CV_8UC1)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::getShapesClearBorder : \
        Inappropriate image type.");
      #endif

      return;
    }
    #ifdef DEBUG_SHOW
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    if(Parameters::Debug::show_get_shapes_clear_border) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Before clear blorders";
      msgs.push_back(msg);
      cv::Mat tmp;
      inImage->copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif
    #ifdef DEBUG_TIME
    Timer::start("getShapesClearBorder", "denoiseEdges");
    #endif

    // Kernel for obtaining boundary pixels
    static const char kernels[8][3][3] = {
      { {0, 2, 2},
        {2, 1, 2},
        {2, 2, 2} },

      { {2, 0, 2},
        {2, 1, 2},
        {2, 2, 2} },

      { {2, 2, 0},
        {2, 1, 2},
        {2, 2, 2} },

      { {2, 2, 2},
        {2, 1, 0},
        {2, 2, 2} },

      { {2, 2, 2},
        {2, 1, 2},
        {2, 2, 0} },

      { {2, 2, 2},
        {2, 1, 2},
        {2, 0, 2} },

      { {2, 2, 2},
        {2, 1, 2},
        {0, 2, 2} },

      { {2, 2, 2},
        {0, 1, 2},
        {2, 2, 2} }
    };

    // Invert the image to facilitate floodfill's operation
    for (unsigned int rows = 0; rows < inImage->rows; rows++)
    {
      for (unsigned int cols = 0; cols < inImage->cols; cols++)
      {
        if (inImage->at<unsigned char>(rows, cols) == 0)
        {
          inImage->at<unsigned char>(rows, cols) = 255;
        }
        else
        {
          inImage->at<unsigned char>(rows, cols) = 0;
        }
      }
    }


    // A vector that holds floodfill images needed for the final output
    std::vector<cv::Mat> floodfillsVector;

    // First floodfill operation
    cv::floodFill(*inImage, cv::Point(0, 0), 0);

    // Push the first floodfill operation's result
    // back into the floodfillsVector
    cv::Mat firstFloodFill;
    inImage->copyTo(firstFloodFill);
    floodfillsVector.push_back(firstFloodFill);


    // Termination flag
    bool isFloodFillFinished = false;

    // The total number of images needed to obtain a clear result
    int numImages = 0;

    while(!isFloodFillFinished)
    {
      // Invert the image to facilitate floodfill's operation
      for (unsigned int rows = 0; rows < inImage->rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage->cols; cols++)
        {
          if (inImage->at<unsigned char>(rows, cols) == 0)
          {
            inImage->at<unsigned char>(rows, cols) = 255;
          }
          else
          {
            inImage->at<unsigned char>(rows, cols) = 0;
          }
        }
      }

      // Second floodfill operation
      cv::floodFill(*inImage, cv::Point(0, 0), 0);


      // The prunedFloodFill image will act as termination check
      // If it is filled with zeros, that means that there are no
      // closed shapes in the floodfill's image.
      // If not, we need to iterate the floodfill procedure
      cv::Mat prunedFloodFill;
      inImage->copyTo(prunedFloodFill);
      Morphology::pruningStrictIterative(&prunedFloodFill, 1000);

      // Check the prunedFloodFill image for the numerosity of zeros
      // If there are non-zero pixels, reiterate
      cv::Mat pruneDiff;
      cv::compare(prunedFloodFill, cv::Mat::zeros(inImage->size(), CV_8UC1),
        pruneDiff, cv::CMP_NE);

      if (cv::countNonZero(pruneDiff) != 0)
      {
        numImages++;

        // Push the second floodfill operation's result
        // back into the floodfillsVector
        cv::Mat nFloodFill;
        inImage->copyTo(nFloodFill);
        floodfillsVector.push_back(nFloodFill);
      }
      else
      {
        isFloodFillFinished = true;
      }
    }

    // The final floodfill-ed image
    cv::Mat finalFloodFill = floodfillsVector[0];
    bool sign = false;
    for (int i = 2; i < floodfillsVector.size(); i += 2)
    {
      finalFloodFill = finalFloodFill +
        (sign == true ? 1 : -1) * floodfillsVector[i];
      sign = (sign == false);
    }

    // Dilate twice:
    // once to (a) get rid of pesky borders
    // and once (b) for the blob's outline to approximate the original's outline
    Morphology::dilation(&finalFloodFill, 2);

    // The floodfill's edges - region borders
    cv::Mat bordersImage = cv::Mat(inImage->rows, inImage->cols, CV_8UC1,
      cv::Scalar::all(0));

    for (int kernelId = 0; kernelId < 8; kernelId++)
    {
      for (unsigned int rows = 1; rows < inImage->rows - 1; rows++)
      {
        for (unsigned int cols = 1; cols < inImage->cols - 1; cols++)
        {
          if (finalFloodFill.at<unsigned char>(rows, cols) != 0 &&
            bordersImage.at<unsigned char>(rows, cols) == 0)
          {
            if (Morphology::kernelCheck(kernels[kernelId], finalFloodFill,
                cv::Point(cols, rows)))
            {
              bordersImage.at<unsigned char>(rows, cols) = 255;
            }
          }
        }
      }
    }

    bordersImage.copyTo(*inImage);

    #ifdef DEBUG_TIME
    Timer::tick("getShapesClearBorder");
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_get_shapes_clear_border) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After clear borders";
      msgs.push_back(msg);
      cv::Mat tmp;
      inImage->copyTo(tmp);
      imgs.push_back(tmp);
    }
    if(Parameters::Debug::show_get_shapes_clear_border) // Debug
    {
      Visualization::multipleShow("getShapesClearBorder function", imgs, msgs,
        1200, 1);
    }
    #endif
  }



  /**
    @brief With an binary input image (quantized in 0 and 255 levels),
    this function fills closed regions, at first, and then extracts the
    outermost outline of each region.
    Used when there is a closed region with garbage pixels with
    a value of 255 within it.
    Caution: Only the outermost outline of shapes is computed. If there are
    closed shapes inside of other closed shapes, only the latter's outline
    will be computed, as opposed to the getShapesClearBorder function
    @param[in,out] inImage [cv::Mat*] The input image
    @return void
   **/
  void EdgeDetection::getShapesClearBorderSimple (cv::Mat* inImage)
  {
    if (inImage->type() != CV_8UC1)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::getShapesClearBorderSimple : \
        Inappropriate image type.");
      #endif

      return;
    }
    #ifdef DEBUG_SHOW
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    if(Parameters::Debug::show_get_shapes_clear_border) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Before clear blorders";
      msgs.push_back(msg);
      cv::Mat tmp;
      inImage->copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::start("getShapesClearBorderSimple", "denoiseEdges");
    #endif

    cv::Mat floodFilledImage;
    inImage->copyTo(floodFilledImage);

    // Flood the original image from (0, 0) with a value of 255
    cv::floodFill(floodFilledImage, cv::Point(0, 0), 255);

    // The pixels affected by the above floodfill operation are
    // backgroung pixels. The ones that remain unaffected are interior to the
    // shapes whose outline we wish to identify.
    // Store these pixels in the tempImage image
    cv::Mat tempImage = cv::Mat::zeros(inImage->size(), CV_8UC1);

    for (int rows = 0; rows < inImage->rows; rows++)
    {
      for (int cols = 0; cols < inImage->cols; cols++)
      {
        if (floodFilledImage.at<unsigned char>(rows, cols) ==
          inImage->at<unsigned char>(rows, cols))
        {
          tempImage.at<unsigned char>(rows, cols) = 255;
        }
      }
    }


    static const char kernels[8][3][3] = {
      { {0, 2, 2},
        {2, 1, 2},
        {2, 2, 2} },

      { {2, 0, 2},
        {2, 1, 2},
        {2, 2, 2} },

      { {2, 2, 0},
        {2, 1, 2},
        {2, 2, 2} },

      { {2, 2, 2},
        {2, 1, 0},
        {2, 2, 2} },

      { {2, 2, 2},
        {2, 1, 2},
        {2, 2, 0} },

      { {2, 2, 2},
        {2, 1, 2},
        {2, 0, 2} },

      { {2, 2, 2},
        {2, 1, 2},
        {0, 2, 2} },

      { {2, 2, 2},
        {0, 1, 2},
        {2, 2, 2} }
    };

    // The image whose non-zero pixels indicate the outlines of shapes
    cv::Mat bordersImage = cv::Mat(inImage->rows, inImage->cols, CV_8UC1,
        cv::Scalar::all(0));

    for (int kernelId = 0; kernelId < 8; kernelId++)
    {
      for (unsigned int rows = 1; rows < inImage->rows - 1; rows++)
      {
        for (unsigned int cols = 1; cols < inImage->cols - 1; cols++)
        {
          if (tempImage.at<unsigned char>(rows, cols) != 0)
          {
            if (Morphology::kernelCheck(kernels[kernelId], tempImage,
                  cv::Point(cols, rows)))
            {
              bordersImage.at<unsigned char>(rows, cols) = 255;
            }
          }
        }
      }
    }

    bordersImage.copyTo(*inImage);

    #ifdef DEBUG_TIME
    Timer::tick("getShapesClearBorderSimple");
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_get_shapes_clear_border) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : After clear borders";
      msgs.push_back(msg);
      cv::Mat tmp;
      inImage->copyTo(tmp);
      imgs.push_back(tmp);
    }
    if(Parameters::Debug::show_get_shapes_clear_border) // Debug
    {
      Visualization::multipleShow("getShapesClearBorderSimple function",
        imgs, msgs, 1200, 1);
    }
    #endif
  }



  /**
    @brief This method takes as input a RGB image and uses
    its backprojection (which is based on the precalculated
    histogram @param inHistogram) in order to identify whole regions whose
    histogram matches @param inHistogram, although the backprojection image
    might be sparcely populated. After the identification of the regions
    of interest, this method extracts their edges and returns the image
    depicting them.
    @param[in] inImage [const cv::Mat&] The input RGB image
    @param[in] inHistogram [const cv::MatND&] The model histogram needed
    in order to obtain the backprojection of @param inImage
    @param[out] outImage [cv::Mat*] The output edges image, in CV_8UC1
    format
    @return void
   **/
  void EdgeDetection::produceEdgesViaBackprojection (
    const cv::Mat& inImage, const cv::MatND& inHistogram, cv::Mat* outImage)
  {
    if (inImage.type() != CV_8UC3)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::produceEdgesViaBackprojection: \
        Inappropriate image type.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("produceEdgesViaBackprojection", "computeRgbEdges");
    #endif

    #ifdef DEBUG_SHOW
    std::string msg;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    #endif

    // Backprojection of the RGB inImage
    cv::Mat backprojectedFrame = cv::Mat::zeros(inImage.size(), CV_8UC1);

    // Get the backprojected image of the frame, based on the precalculated
    // inHistogram histogram
    Histogram::getBackprojection(inImage, inHistogram,
      &backprojectedFrame, Parameters::Histogram::secondary_channel);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      backprojectedFrame.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Backprojection ";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    cv::threshold(backprojectedFrame, backprojectedFrame,
      Parameters::Rgb::compute_edges_backprojection_threshold, 255,
      cv::THRESH_BINARY);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      backprojectedFrame.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Backprojection thresholded to 128"; // should be parameterized
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // The foreground image needed by the watershed algorithm
    cv::Mat foreground = cv::Mat::zeros(inImage.size(), CV_8UC1);

    // Copy the backprojection to the foreground image
    backprojectedFrame.copyTo(foreground);

    // Dilate
    Morphology::dilationRelative(&foreground,
      Parameters::Rgb::watershed_foreground_dilation_factor);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      foreground.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Backprojection dilated by " +
        TOSTR(Parameters::Rgb::watershed_foreground_dilation_factor);
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // All non-zero pixels have now a value of 255
    cv::threshold(foreground, foreground, 0, 255, cv::THRESH_BINARY);

    // Erode. The erosion factor should be greater
    // than the dilation factor used above
    Morphology::erosion(&foreground,
      Parameters::Rgb::watershed_foreground_erosion_factor);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      foreground.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Foreground eroded by " +
        TOSTR(Parameters::Rgb::watershed_foreground_erosion_factor);
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // The background image needed by the watershed algorithm
    cv::Mat background = cv::Mat::zeros(inImage.size(), CV_8UC1);

    // Copy the backprojection to the background image
    backprojectedFrame.copyTo(background);

    // Dilate
    Morphology::dilationRelative(&background,
      Parameters::Rgb::watershed_background_dilation_factor);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      background.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Backprojection dilated by " +
        TOSTR(Parameters::Rgb::watershed_background_dilation_factor);
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // All non-zero pixels have now a value of 255
    cv::threshold(background, background, 0, 255, cv::THRESH_BINARY);

    // All zero value pixels turn to white, all white to black
    cv::threshold(background, background, 0, 255, cv::THRESH_BINARY_INV);

    // Erode. The erosion factor should be greater
    // than the dilation factor used above. This erosion happens so that
    // the background pixels belong surely to the background
    Morphology::erosion(&background,
      Parameters::Rgb::watershed_background_erosion_factor);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      background.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Background eroded by " +
        TOSTR(Parameters::Rgb::watershed_background_erosion_factor);
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // All zero value pixels' values are elevated to 128.
    // These belong neither to foreground, nor to background:
    // they are labeled as so-called "unknown"
    cv::threshold(background, background, 0, 128, cv::THRESH_BINARY);

    // Create the markers image, needed by the watershed algorighm
    cv::Mat markers(inImage.size(), CV_8UC1, cv::Scalar(0));

    // The markers array is composed by the foreground, background and
    // unknown pixels
    markers = foreground + background;

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      background.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Markers";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // Convert the marker image of type CV_8UC1, to CV_32S
    cv::Mat markers32S;
    markers.convertTo(markers32S, CV_32S);

    // Watershed the input image, with regard to the markers constructed
    // Each pixel p is transformed into
    // 255p + 255 before conversion
    cv::watershed(inImage, markers32S);

    // Convert the markers image to back to type CV_8UC1.
    // This image identifies the whole area that matches the histogram
    // inHistogram in the input image
    markers32S.convertTo(*outImage, CV_8U, 255, 255);

    // All zero value pixels turn to white, all white to black.
    // In essence, this is the edges of the area whose histogram
    // matches inHistogram
    cv::threshold(*outImage, *outImage, 0, 255, cv::THRESH_BINARY_INV);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      outImage->copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Edges from watershed";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      Visualization::multipleShow("Final edges", imgs, msgs,
        Parameters::Debug::show_produce_edges_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("produceEdgesViaBackprojection");
    #endif
  }



  /**
    @brief This method takes as input a RGB image, segments it,
    and extracts its edges.
    @param[in] inImage [const cv::Mat&] The input RGB image,
    of type CV_8UC3
    @param[in] segmentationMethod [const int&] The method by which the
    segmentation of @param inImage will be performed.
    @param[out] outImage [cv::Mat*] The output edges image,
    of type CV_8UC1
    @return void
   **/
  void EdgeDetection::produceEdgesViaSegmentation (const cv::Mat& inImage,
    const int& segmentationMethod, cv::Mat* edges)
  {
    if (inImage.type() != CV_8UC3)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::produceEdgesViaSegmentation : \
        Inappropriate image type.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("produceEdgesViaSegmentation", "computeRgbEdges");
    #endif

    #ifdef DEBUG_SHOW
    std::string msg;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_produce_edges) // Debug
    {
      cv::Mat tmp;
      inImage.copyTo(tmp);
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial RGB image";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // Copy the input image to the segmentedHoleFrame one
    cv::Mat segmentedHoleFrame = cv::Mat(inImage.size(), CV_8UC3);
    inImage.copyTo(segmentedHoleFrame);

    // Segment the input image.
    segmentation(inImage, segmentationMethod, &segmentedHoleFrame);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      segmentedHoleFrame.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Segmented RGB image";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // Posterize the product of the segmentation
    if (Parameters::Rgb::posterize_after_segmentation)
    {
      // Fill the various segments with colour
      floodFillPostprocess(&segmentedHoleFrame);

      #ifdef DEBUG_SHOW
      if (Parameters::Debug::show_produce_edges)
      {
        cv::Mat tmp;
        segmentedHoleFrame.copyTo(tmp);
        msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
        msg += " : Posterized segmented RGB image";
        msgs.push_back(msg);
        imgs.push_back(tmp);
      }
      #endif
    }

    // Convert the posterized image to grayscale
    cv::Mat segmentedHoleFrame8UC1 =
      cv::Mat::zeros(segmentedHoleFrame.size(), CV_8UC1);

    // In order to find the edges of the posterized image,
    // first, turn it to grayscale
    cv::cvtColor(segmentedHoleFrame, segmentedHoleFrame8UC1, CV_BGR2GRAY);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      segmentedHoleFrame8UC1.copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Segmented RGB image in grayscale";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // Apply edge detection to the grayscale posterized input RGB image
    if (Parameters::Edge::edge_detection_method == 0)
    {
      EdgeDetection::applyCanny(segmentedHoleFrame8UC1, edges);
    }
    else if (Parameters::Edge::edge_detection_method == 1)
    {
      EdgeDetection::applyScharr(segmentedHoleFrame8UC1, edges);
    }
    else if (Parameters::Edge::edge_detection_method == 2)
    {
      EdgeDetection::applySobel(segmentedHoleFrame8UC1, edges);
    }
    else if (Parameters::Edge::edge_detection_method == 3)
    {
      EdgeDetection::applyLaplacian(segmentedHoleFrame8UC1, edges);
    }
    else if (Parameters::Edge::edge_detection_method == 4) // Mixed mode
    {
      if (Parameters::Edge::mixed_edges_toggle_switch == 1)
      {
        EdgeDetection::applyScharr(segmentedHoleFrame8UC1, edges);
        Parameters::Edge::mixed_edges_toggle_switch = 2;
      }
      else if (Parameters::Edge::mixed_edges_toggle_switch == 2)
      {
        EdgeDetection::applySobel(segmentedHoleFrame8UC1, edges);
        Parameters::Edge::mixed_edges_toggle_switch = 1;
      }
    }

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      cv::Mat tmp;
      edges->copyTo(tmp);
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Edges from the segmented grayscale image";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_produce_edges)
    {
      Visualization::multipleShow("Final edges", imgs, msgs,
        Parameters::Debug::show_produce_edges_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("produceEdgesViaSegmentation");
    #endif
  }



  /**
    @brief Segments a RGB image
    @param[in] inImage [const cv::Mat&] The RGB image to be segmented
    @param[in] segmentationMethod [const int&] The method by which the
    segmentation of @param inImage will be performed.
    @param[out] outImage [cv::Mat*] The posterized image
    @return void
   **/
  void EdgeDetection::segmentation(const cv::Mat& inImage,
    const int& segmentationMethod, cv::Mat* outImage)
  {
    if (inImage.type() != CV_8UC3)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("EdgeDetection::segmentation: Inappropriate image type.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("segmentation", "produceEdgesViaSegmentation");
    #endif

    // Blur the input image
    if (segmentationMethod == 0)
    {
      // Segment the image
      cv::pyrMeanShiftFiltering(inImage, *outImage,
        Parameters::Rgb::spatial_window_radius,
        Parameters::Rgb::color_window_radius,
        Parameters::Rgb::maximum_level_pyramid_segmentation);
    }
    else if (segmentationMethod == 1)
    {
      for ( int i = 1; i < 15; i = i + 4 )
      {
        cv::medianBlur(inImage, *outImage, i);
      }
    }

      #ifdef DEBUG_TIME
      Timer::tick("segmentation");
      #endif
  }

} // namespace pandora_vision

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
 * Authors: Vasilis Bosdelekidis, Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#include "depth_node/depth_processor.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Constructor
   **/
  DepthProcessor::DepthProcessor(
      const std::string& ns, 
      sensor_processor::Handler* handler) 
    : VisionProcessor(ns, handler)
  {
    ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
        this->accessProcessorNh()->getNamespace());
    DepthParametersHandler_ = new DepthParametersHandler();
  }


  DepthProcessor::DepthProcessor() : VisionProcessor() {}


  /**
    @brief Destructor
   **/
  DepthProcessor::~DepthProcessor()
  {
    ROS_INFO_NAMED(PKG_NAME, "[Depth Processor] Terminated");
  }


  /**
    @brief Finds holes, provided a depth image in CV_32FC1 format.

    1. some basic eroding and dilation to eliminate isolated noise pixels.
    2. eliminate huge contours and tiny contours without filtering. There is no problem with the huge and no problem with the tiny for distances less than 2.5m (in any case, in such distances holes are not obvious at all).
    3. Eliminate contours which have an over 4x bounding box height/ width fraction or the inverse. There was no TP loss by this progress, instead wall edges were eliminated.
    4. Merge contours. Check only in small distance. Firstly label contours to merge together with a probability. The probability consists of a weighted sum of some features inside a box between the two contours' keypoints. Currently, features are the inverse of the euclidean distance between keypoints and the sum of blacks in the original image. 
    5. The contours checked at each step are those that were not eliminated by previous steps.
    @param[in] depthImage [const cv::Mat&] The depth image in CV_32FC1 format
    @return HolesConveyor The struct that contains the holes found
   **/
  HolesConveyor DepthProcessor::findHoles(const cv::Mat& depthImage) 
  {
    //#ifdef DEBUG_TIME
    //    Timer::start("findHoles", "inputRgbImageCallback");
    //#endif
    //
#ifdef DEBUG_SHOW
    std::string msg;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
#endif

#ifdef DEBUG_SHOW
    if(Depth::show_find_holes) // Debug
    {
      cv::Mat tmp;
      depthImage.copyTo(tmp);
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial depth image";
      msgs.push_back(msg);
      tmp = Visualization::scaleImageForVisualization(
          depthImage, 0);
      imgs.push_back(tmp);
    }
#endif

    HolesConveyor conveyor;
    // do not even try to find holes if the distance is too small or equivalently the frame is too noisy
    float sum = 0;
    for(int row = 0; row < depthImage.rows; row ++)
      for(int col = 0; col < depthImage.cols; col ++)
      {
        sum += depthImage.at<float>(row, col);
      }
    float avg = sum / (depthImage.rows * depthImage.cols);
    if(avg > Depth::min_valid_depth)
    {

      // Initial filtering of contours variant from the background
      cv::Mat filteredImage;
      filterImage(
          depthImage,
          &filteredImage);

#ifdef DEBUG_SHOW
      if(Depth::show_find_holes) // Debug
      {
        cv::Mat tmp;
        filteredImage.copyTo(tmp);
        std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
        msg += STR(" : Edges after filtering");
        msgs.push_back(msg);
        imgs.push_back(tmp);
      }
#endif

      std::vector<std::vector<cv::Point> > contours;
      // Find contours in the variance image.
      detectContours(filteredImage, &contours);
      // center of mass of each contour
      std::vector<cv::Point2f> mc(contours.size());
      std::vector<cv::Rect> boundRect(contours.size());
      // Get center of mass and bounding box of each contour.
      getContourInfo(contours , &mc, &boundRect);
      std::vector<bool> realContours(contours.size(), true);
      // True contour sizes after possible merging
      std::vector<int> contourWidth(contours.size());
      std::vector<int> contourHeight(contours.size());
      // Validate contours found. The product is a vector with a flag for each contour.
      validateContours(
          depthImage, 
          contours, 
          &mc, 
          &contourHeight, 
          &contourWidth, 
          &realContours, 
          boundRect);
      // The final vectors of keypoints, and rectangles.
      std::vector<cv::Point2f> keypoints;
      std::vector<cv::Rect> rectangles;
      std::vector<std::vector<cv::Point> > outlines;
      for(int i = 0; i < contours.size(); i++)
        if(realContours.at(i))
        {
          keypoints.push_back(mc[i]);
          cv::Rect temp(
              mc[i].x - contourWidth[i] / 2, 
              mc[i].y - contourHeight[i] / 2, 
              contourWidth[i], 
              contourHeight[i]);
          rectangles.push_back(temp);
          outlines.push_back(contours[i]);
        }

      conveyor.keypoint = keypoints;
      conveyor.rectangle = rectangles;
      conveyor.outline = outlines;

#ifdef DEBUG_SHOW
      if(Depth::show_find_holes) // Debug
      {
        std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
        msg += STR(" : Initial keypoints");
        msgs.push_back(msg);
        imgs.push_back(Visualization::showKeypoints(msg, filteredImage, -1, mc));
      }
#endif


#ifdef DEBUG_SHOW
      if (Depth::show_find_holes)
      {
        msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
        msg += STR(" : Blobs");
        msgs.push_back(msg);
        imgs.push_back(
            Visualization::showHoles(
              msg,
              depthImage,
              conveyor,
              -1,
              std::vector<std::string>())
            );
      }
#endif

#ifdef DEBUG_SHOW
      if (Depth::show_find_holes)
      {
        Visualization::multipleShow("Depth node", imgs, msgs,
            Depth::show_find_holes_size, 1);
      }
#endif
    }
    else 
    {
      conveyor.rectangle.clear();
      conveyor.keypoint.clear();
    }

    //#ifdef DEBUG_TIME
    //    Timer::tick("findHoles");
    //#endif

    return conveyor;
  }


  /**
    @brief The function called to filter the depth image
    @details Based on filtering_type param applying simple thresholding at 0, simple morhology transformations and to eliminate (make dark) the region at borders or simple edge detection method.
    @param[in] depthImage [const cv::Mat&] The Depth image to be processed,
    in CV_8UC3 format
    @param[in] filteredImage [cv::Mat* filteredImage] The output filtered binary image.
    @return void
   **/
  void DepthProcessor::filterImage(
      const cv::Mat& depthImage, 
      cv::Mat* filteredImage)
  {
    // The input depth image, in CV_8UC1 format
    cv::Mat visualizableDepthImage = 
      Visualization::scaleImageForVisualization(
          depthImage,
          0);
    if(Depth::filtering_type == 0)
    {
      cv::threshold(
          visualizableDepthImage, 
          (*filteredImage), 
          Depth::intensity_threshold, 
          1.0, 
          CV_THRESH_BINARY);
      int morphologyKernel = Depth::morphology_open_kernel_size;
      cv::Mat structuringElement = 
        cv::getStructuringElement(
            cv::MORPH_ELLIPSE, 
            cv::Size(morphologyKernel, morphologyKernel));
      cv::morphologyEx( 
          (*filteredImage), 
          (*filteredImage), 
          cv::MORPH_OPEN, 
          structuringElement );
      morphologyKernel = Depth::morphology_close_kernel_size;
      structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morphologyKernel, morphologyKernel));
      cv::morphologyEx((*filteredImage), (*filteredImage), cv::MORPH_CLOSE, structuringElement);
    }
    else
    {
      cv::Mat temp; 
      cv::Canny(
          visualizableDepthImage, 
          temp, 
          Depth::canny_low_threshold, 
          Depth::canny_low_threshold * Depth::canny_ratio, 
          Depth::canny_kernel_size);
      //apply canny mask
      visualizableDepthImage.copyTo((*filteredImage), temp);
    }
    for(int i = 0; i < Depth::border_thresh; i ++)
      for(int j = 0; j < (*filteredImage).cols; j ++)
        (*filteredImage).at<uchar>(i, j) = 0;
    for(int i = (*filteredImage).rows - Depth::border_thresh; i < (*filteredImage).rows; i ++)
      for(int j = 0; j < (*filteredImage).cols; j ++)
        (*filteredImage).at<uchar>(i, j) = 0;
    for(int i = 0; i < (*filteredImage).rows; i ++)
      for(int j = 0; j < Depth::border_thresh; j ++)
        (*filteredImage).at<uchar>(i, j) = 0;
    for(int i = 0; i < (*filteredImage).rows; i ++)
      for(int j = (*filteredImage).cols - Depth::border_thresh; j < (*filteredImage).cols; j ++)
        (*filteredImage).at<uchar>(i, j) = 0;
  }


  /**
    @brief The function called to extract contours that were not ignored by the initial filtering held in filterImage function.
    @param[in] filteredImage [cv::Mat&] The filtered image represented as white edges where there was a small depth. Remember that there were only edges left, which were black at the original depth image and were not random black pixels.
    @param[in] contours [std::vector<std::vector<cv::Point>>*] The contours found.
    @return void
   **/
  void DepthProcessor::detectContours(
      const cv::Mat& filteredImage, 
      std::vector<std::vector<cv::Point> >* contours)
  {
    cv::Mat temp;
    int dilationKernel = Depth::dilation_kernel_size;
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilationKernel, dilationKernel));
    cv::dilate(filteredImage, filteredImage, structuringElement);
    cv::vector<cv::Vec4i> hierarchy;
    //cv::Mat temp1;
    //if (filteredImage.type() != CV_8UC1 && filteredImage.type() != CV_8UC3)
    //  filteredImage.convertTo(temp1, CV_8UC1);
    //else
    //  filteredImage.copyTo(temp1);
    findContours(filteredImage, (*contours), hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  }


  /**
    @brief The function called to estimate center of mass for each contour found and bounding boxes.
    @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
    @param[in] mc [std::vector<cv::Point2f>*] Center of mass of each contour as x, y coordinates..
    @return void
   **/
  void DepthProcessor::getContourInfo(
      const std::vector<std::vector<cv::Point> >& contours, 
      std::vector<cv::Point2f>* mc, 
      std::vector<cv::Rect>* boundRect)
  {
    std::vector<std::vector<cv::Point> > contoursPoly(contours.size());
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());
    std::vector<cv::Moments> mu(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {
      mu[i] = moments(contours[i], false);
      (*mc)[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00); 
      cv::approxPolyDP(cv::Mat(contours[i]), contoursPoly[i], 3, true);
      (*boundRect)[i] = cv::boundingRect(cv::Mat(contoursPoly[i]));
      cv::minEnclosingCircle((cv::Mat)contoursPoly[i], center[i], radius[i]);
    }
  }


  /**
    @brief The function called to make validation of found contours
    @param[in] image [cv::Mat&] The original image. Used to extract some features to evaluate contour size, similarity etc.
    @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
    @param[in] mc [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
    @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
    @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
    @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
    @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
    @return void
   **/
  void DepthProcessor::validateContours(
      const cv::Mat& image, 
      const std::vector<std::vector<cv::Point> >& contours, 
      std::vector<cv::Point2f>* mc, 
      std::vector<int>* contourHeight, 
      std::vector<int>* contourWidth, 
      std::vector<bool>* realContours, 
      const std::vector<cv::Rect>& boundRect)
  {
    std::vector<int> numLabels(contours.size(), 0);
    // for merging, contour index, other contour index, possibility
    std::map<std::pair<int, int>, float> contourLabel;
    for(int i = 0; i < contours.size(); i ++)
    {
      (*contourHeight)[i] = boundRect[i].height;
      (*contourWidth)[i] = boundRect[i].width;
      numLabels[i] = 0;
      if((*realContours)[i])
      {
        (*realContours)[i] = 
          validateContour(
              image, 
              i, 
              mc, 
              contourHeight, 
              contourWidth, 
              &contourLabel, 
              &numLabels, 
              boundRect, 
              contours, 
              realContours);
        if((*realContours).at(i))
          if((*contourWidth)[i] > Depth::rect_diff_thresh * (*contourHeight)[i] 
              || (*contourHeight)[i] > Depth::rect_diff_thresh * (*contourWidth)[i])
            (*realContours)[i] = false;
      }
    }
    for( int i = 0; i < contours.size(); i++ )
    {
      if((*realContours)[i] && numLabels[i] > 0)
        mergeContours(i, mc, contourHeight, contourWidth, &contourLabel, &numLabels, realContours, contours);
    }
  }


  /**
    @brief The function called by validateContours to make validation of a single contour
    @param[in] image [const cv::Mat&] The depth image 
    @param[in] ci [int] Current contour index in contours vector 
    @param[in] mcv [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
    @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
    @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
    @param[in] contourLabel [std::map<std::pair<int, int>, float>*] A map of contours relationship represented as current contour & some other contour as key and a probability of shame contour calculated via the values of some features between the two contours.
    @param[in] numLabels [std::vector<int>*] For each contour a counter of how many times it appears in the abovementioned map. In the map are strored only pairs of contours whose merging probability is above some threshold.
    @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
    @param[in] contours [std::vector<std::vector<cv::Point> >&] All contours found. 
    @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. 
    @return void
   **/
  bool DepthProcessor::validateContour(
      const cv::Mat& image, 
      int ci, 
      std::vector<cv::Point2f>* mcv, 
      std::vector<int>* contourHeight, 
      std::vector<int>* contourWidth, 
      std::map<std::pair<int, int>, float>* contourLabel, 
      std::vector<int>* numLabels, 
      const std::vector<cv::Rect>& boundRect, 
      const std::vector<std::vector<cv::Point> >& contours, 
      std::vector<bool>* realContours)
  {
    int sumWhites = 0;
    if(cv::contourArea(contours[ci]) > Depth::huge_contour_thresh 
        || cv::contourArea(contours[ci]) < Depth::tiny_contour_thresh)
      return false;
    else
    {
      if(Depth::shape_validation)
        validateShape(image, contours, boundRect, ci, &(*realContours));
      if((*realContours)[ci])
      {
        for(int i = 0; i < contours.size(); i ++)
        {
          if(i != ci)
          {
            if((*realContours)[i] 
                && (std::abs((*mcv)[ci].x - (*mcv)[i].x) < Depth::neighbor_thresh) 
                && (std::abs((*mcv)[ci].y - (*mcv)[i].y) < Depth::neighbor_thresh) 
                && (std::abs(static_cast<int>(image.at<uchar>((*mcv)[ci].y, (*mcv)[ci].x)) 
                    - static_cast<int>(image.at<uchar>((*mcv)[i].y, (*mcv)[i].x))) 
                  < Depth::neighbor_value_thresh))
            {
              int upperX;
              int upperY;
              int lowerX;
              int lowerY;
              if((*mcv)[ci].x > (*mcv)[i].x)
                upperX = (*mcv)[i].x;
              else
                upperX = (*mcv)[ci].x;
              if((*mcv)[ci].y> (*mcv)[i].y)
                upperY = (*mcv)[i].y;
              else
                upperY = (*mcv)[ci].y;
              if((*mcv)[ci].x > (*mcv)[i].x)
                lowerX = (*mcv)[ci].x;
              else
                lowerX = (*mcv)[i].x;
              if((*mcv)[ci].y > (*mcv)[i].y)
                lowerY = (*mcv)[ci].y;
              else
                lowerY = (*mcv)[i].y;
              cv::Mat ROI = image(boundRect[i]);
              cv::Scalar otherAvg = cv::mean(ROI);
              int homogRectWidth = std::abs(lowerX - upperX);
              if(homogRectWidth < Depth::depth_similarity_rect_dims_thresh)
                homogRectWidth = Depth::depth_similarity_rect_dims_thresh;
              int homogRectHeight = std::abs(lowerY - upperY);
              if(homogRectHeight < Depth::depth_similarity_rect_dims_thresh)
                homogRectHeight = Depth::depth_similarity_rect_dims_thresh;
              if(upperX + homogRectWidth > image.cols)
                homogRectWidth = image.cols - upperX - 1;
              if(upperY + homogRectHeight > image.rows)
                homogRectHeight = image.rows - upperY - 1;
              if(upperX < 0)
                upperX = 0;
              if(upperY < 0)
                upperY = 0;
              if(lowerX > image.cols)
                lowerX = image.cols;
              if(lowerY > image.rows)
                lowerY = image.rows;
              //cout << upperX << ", " << upperY << ", " << homogRectWidth << ", " << homogRectHeight << "\n";
              ROI = image(cv::Rect(upperX, upperY, homogRectWidth, homogRectHeight));
              int sumBlacks = 0;
              for(int r = 0; r < ROI.rows; r ++)
                for(int c = 0; c < ROI.cols; c ++)
                  if(static_cast<int>(image.at<uchar>(r, c)) == 0)
                    sumBlacks++;
              float euclideanDistance = 
                1 / (sqrt(pow((*mcv)[i].x - (*mcv)[ci].x, 2) 
                      + pow((*mcv)[i].x - (*mcv)[ci].x, 2)));
              float mergeProbability = sumBlacks * 0.5 + euclideanDistance * 0.5;
              //std::cout << mergeProbability << "\n";
              if(mergeProbability > Depth::merge_thresh)
              {
                if((*contourLabel).find(std::make_pair(i, ci)) == (*contourLabel).end())
                {
                  (*contourLabel)[std::make_pair(ci, i)] = mergeProbability;
                  (*numLabels)[ci]++;
                  (*numLabels)[i]++;
                }
              }
            }
          }
        }
        int newIndX = (*mcv)[ci].x;
        int newIndY = (*mcv)[ci].y;
        return true;
      }
      return false;
    }
  }

  void DepthProcessor::validateShape(
      const cv::Mat& image, 
      const std::vector<std::vector<cv::Point> >& outline, 
      const std::vector<cv::Rect>& boundRect, 
      int ci, 
      std::vector<bool>* realContours)
  {
    // Draw contour on a black frame with white color
    cv::Mat canvas = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    cv::Scalar color = cv::Scalar(255, 255, 255);
    cv::drawContours(canvas, outline, ci, color);
    int intersectionX = 0;
    int intersectionY = 0;
    int pixelsInside = 0;
    int maxIntersectionsX = 0;
    int maxIntersectionsY = 0;
    for(int row = boundRect[ci].y; row < (boundRect[ci].y + boundRect[ci].height); row ++)
    {
      bool isInside = false;
      bool isOutline = false;
      int intersectionCurrent = 0;
      for(int col = boundRect[ci].x; col < (boundRect[ci].x + boundRect[ci].width); col ++)
      {
        if(canvas.at<int>(row, col) == 255)
        {
          if(!isOutline)
          {
            intersectionX++;
            intersectionCurrent++;
            isOutline = true;
          }
        } 
        else
        {
          if(intersectionCurrent % 2 != 0)
          {
            isInside = true;
            pixelsInside++;
          }
          else
            isInside = false;
          isOutline = false;
        }
      }
      if(intersectionCurrent > maxIntersectionsX)
        maxIntersectionsX = intersectionCurrent;
    }

    float avgPixelsInsideX = pixelsInside / boundRect[ci].height; 
    pixelsInside = 0;

    for(int col = boundRect[ci].x; col < (boundRect[ci].x + boundRect[ci].width); col ++)
    {
      bool isInside = false;
      bool isOutline = false;
      int intersectionCurrent = 0;
      for(int row = boundRect[ci].y; row < (boundRect[ci].y + boundRect[ci].height); row ++)
      {
        if(canvas.at<int>(row, col) == 255)
        {
          if(!isOutline)
          {
            intersectionY++;
            intersectionCurrent++;
            isOutline = true;
          }
        } 
        else
        {
          if(intersectionCurrent % 2 != 0)
          {
            isInside = true;
            pixelsInside++;
          }
          else
            isInside = false;
          isOutline = false;
        }
      }
      if(intersectionCurrent > maxIntersectionsY)
        maxIntersectionsY = intersectionCurrent;
    }

    float avgPixelsInsideY = pixelsInside / boundRect[ci].width; 

    if((boundRect[ci].height / avgPixelsInsideY >= 
          Depth::one_direction_rectangle_contour_overlap_thresh 
          || boundRect[ci].width / avgPixelsInsideX >= 
          Depth::one_direction_rectangle_contour_overlap_thresh)
        && (maxIntersectionsX > Depth::max_intersections_thresh
          || maxIntersectionsY > Depth::max_intersections_thresh))
      (*realContours)[ci] = false;

  }

  /**
    @brief The function called to make validation of found contours
    @param[in] image [cv::Mat&] The original image. Used to extract some features to evaluate contour size, similarity etc.
    @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
    @param[in] mc [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
    @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
    @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
    @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
    @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
    @return void
   **/
  void DepthProcessor::mergeContours(
      int ci, 
      std::vector<cv::Point2f>* mcv, 
      std::vector<int>* contourHeight, 
      std::vector<int>* contourWidth, 
      std::map<std::pair<int, int>, float>* contourLabel, 
      std::vector<int>* numLabels, 
      std::vector<bool>* realContours, 
      const std::vector<std::vector<cv::Point> >& contours)
  {
    int maxProbability = 0;
    double sumX = (*mcv)[ci].x;
    double sumY = (*mcv)[ci].y;
    double sum = 1;
    for(int i = 0; i < (*contourLabel).size(); i++)
    {
      for(int j = 0; j < contours.size(); j ++)
      {
        if((*realContours).at(j) && j != ci)
          if((*contourLabel).find(std::make_pair(ci, j)) != 
              (*contourLabel).end() 
              || (*contourLabel).find(std::make_pair(j, ci)) != 
              (*contourLabel).end())
          {
            //if((*contourLabel)[ci][j] > max)
            //{
            //    maxProbability = (*contourLabel)[ci][j];
            //}
            (*realContours)[j] = false;
            (*contourWidth)[ci] += (*contourWidth)[j];
            (*contourHeight)[ci] += (*contourHeight)[j];
            sum++;
            sumX += (*mcv)[j].x;
            sumY += (*mcv)[j].y;
            for(int k = 0; k < (*contourLabel).size(); k++)
              for(int l = 0; l < contours.size(); l ++)
                if((*realContours)[l] && l != ci && l != j)
                  if((*contourLabel).find(std::make_pair(j, l)) != 
                      (*contourLabel).end() 
                      || (*contourLabel).find(std::make_pair(l, j)) != 
                      (*contourLabel).end())
                  {
                    //if((*contourLabel)[ci][j] > max)
                    //{
                    //    maxProbability = (*contourLabel)[ci][j];
                    //}
                    (*realContours)[l] = false;
                    (*contourWidth)[ci] += (*contourWidth)[l];
                    (*contourHeight)[ci] += (*contourHeight)[l];
                    sum++;
                    sumX += (*mcv)[l].x;
                    sumY += (*mcv)[l].y;
                  }
          }
      }
    }
    (*mcv)[ci].x = sumX / sum;
    (*mcv)[ci].y = sumY / sum;
  }
  
  
  bool DepthProcessor::process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getImage().cols;
    output->frameHeight = input->getImage().rows;

#ifdef DEBUG_TIME
    Timer::start("depthProcessor", "", true);
#endif

#ifdef DEBUG_SHOW
    if (Depth::show_depth_image)
    {
      Visualization::showScaled("Depth image", input->getImage(), 1);
    }
#endif

    // Locate potential holes in the depth image
    HolesConveyor conveyor = findHoles(input->getImage());

    for (int ii = 0; ii < conveyor.rectangle.size(); ii++)
    {
      if (ii == 0)
      {
        output->pois.clear();
      }
      BBoxPOIPtr bbox(new BBoxPOI);

      bbox->setPoint(conveyor.keypoint[ii]);
      bbox->setWidth(conveyor.rectangle[ii].width);
      bbox->setHeight(conveyor.rectangle[ii].height);

      output->pois.push_back(bbox);
    }

#ifdef DEBUG_TIME
    Timer::tick("depthProcessor");
    Timer::printAllMeansTree();
#endif

    //if (output->pois.empty())
    //{
    //  return false;
    //}
    return true;
  }

}  // namespace pandora_vision

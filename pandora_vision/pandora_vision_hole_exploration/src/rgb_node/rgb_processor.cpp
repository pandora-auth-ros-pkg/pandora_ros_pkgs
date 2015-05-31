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
 * Authors: Vasilis Bosdelekidis, Despoina Paschalidou, Alexandros Philotheou
 *********************************************************************/

#include "rgb_node/rgb_processor.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Constructor
   **/
  RgbProcessor::RgbProcessor(const std::string& ns, sensor_processor::Handler* handler) : VisionProcessor(ns, handler)
  {
    ROS_INFO_STREAM("["+this->getName()+"] processor nh processor : "+
        this->accessProcessorNh()->getNamespace());
    RgbParametersHandler_ = new RgbParametersHandler();
  }


  RgbProcessor::RgbProcessor() : VisionProcessor() {}


  /**
    @brief Destructor
   **/
  RgbProcessor::~RgbProcessor()
  {
    ROS_INFO_NAMED(PKG_NAME, "[Rgb Processor] Terminated");
  }




  /**
    @brief The function called to extract holes from RGB image
    @param[in] rgbImage [const cv::Mat&] The RGB image to be processed,
    in CV_8UC3 format
    @return [HolesConveyor] A struct with useful info about each hole.
   **/
  HolesConveyor RgbProcessor::findHoles(const cv::Mat& rgbImage) 
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
    //
#ifdef DEBUG_SHOW
    if(Rgb::show_find_holes) // Debug
    {
      cv::Mat tmp;
      rgbImage.copyTo(tmp);
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial RGB image";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
#endif

    // Initial filtering of contours variant from the background
    cv::Mat bigVarianceContours;
    computeVarianceImage(
        rgbImage,
        &bigVarianceContours);

#ifdef DEBUG_SHOW
    if(Rgb::show_find_holes) // Debug
    {
      cv::Mat tmp;
      bigVarianceContours.copyTo(tmp);
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Edges after denoise");
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
#endif

    std::vector<std::vector<cv::Point> > contours;
    // Find contours in the variance image.
    detectContours(bigVarianceContours, &contours);
    // center of mass of each contour
    std::vector<cv::Point2f> mc(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    // Get center of mass and bounding box of each contour.
    getContourInfo(contours , &mc, &boundRect);
    std::vector<bool> realContours(contours.size(), true);
    // True contour sizes after possible merging
    std::vector<int> contourWidth(contours.size());
    std::vector<int> contourHeight(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {
      contourHeight[i] = boundRect[i].height; 
      contourWidth[i] = boundRect[i].width; 
    }
    // Validate contours found. The product is a vector with a flag for each contour.
    validateContours(rgbImage, contours , &mc, &contourHeight, &contourWidth, &realContours, boundRect);
    // The final vectors of keypoints, and rectangles.
    HolesConveyor conveyor;
    std::vector<cv::Point2f> keypoints;
    std::vector<cv::Rect> rectangles;
    std::vector<std::vector<cv::Point> > outlines;
    for(int i = 0; i < contours.size(); i++)
    {
      if(realContours.at(i))
      {
        boundRect[i] = 
          cv::Rect(
              mc[i].x - contourWidth[i] / 2, 
              mc[i].y - contourHeight[i] / 2, 
              contourWidth[i], 
              contourHeight[i]);
        keypoints.push_back(mc[i]);
        rectangles.push_back(boundRect[i]);
        outlines.push_back(contours[i]);
      }
    }

    conveyor.keypoint = keypoints;
    conveyor.rectangle = rectangles;
    conveyor.outline = outlines;

#ifdef DEBUG_SHOW
    if(Rgb::show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Initial keypoints");
      msgs.push_back(msg);
      imgs.push_back(Visualization::showKeypoints(msg, bigVarianceContours, -1, keypoints));
    }
#endif

#ifdef DEBUG_SHOW
    if (Rgb::show_find_holes)
    {
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Blobs");
      msgs.push_back(msg);
      imgs.push_back(
          Visualization::showHoles(
            msg,
            rgbImage,
            conveyor,
            -1,
            std::vector<std::string>())
          );
    }
#endif

#ifdef DEBUG_SHOW
    if (Rgb::show_find_holes)
    {
      Visualization::multipleShow("RGB node", imgs, msgs,
          Rgb::show_find_holes_size, 1);
    }
#endif

#ifdef DEBUG_TIME
    Timer::tick("findHoles");
#endif

    return conveyor;
  }


  /**
    @brief The function called to calculate the variance in the RGB image, at overlapping windows of specific size.
    @param[in] rgbImage [const cv::Mat&] The RGB image to be processed,
    in CV_8UC3 format
    @param[in] bigVarianceContours [cv::Mat* bigVarianceContours] The output binary image thresholded based on the variance of the RGB image.
    @return void
   **/
  void RgbProcessor::computeVarianceImage(const cv::Mat& rgbImage, cv::Mat* bigVarianceContours)
  {
    cv::GaussianBlur(rgbImage, rgbImage, cv::Size(0, 0), Rgb::original_image_gaussian_blur);

    cv::Mat image32f;
    rgbImage.convertTo(image32f, CV_32F);

    cv::Mat mu;
    int windowSize = Rgb::std_variance_kernel_size;
    cv::blur(image32f, mu, cv::Size(windowSize, windowSize));

    cv::Mat mu2;
    cv::blur(image32f.mul(image32f), mu2, cv::Size(windowSize, windowSize));

    cv::sqrt(mu2 - mu.mul(mu), (*bigVarianceContours));

    cv::normalize((*bigVarianceContours), (*bigVarianceContours), 0.0, 1.0, cv::NORM_MINMAX);
    if((*bigVarianceContours).channels() == 3)
      cv::cvtColor((*bigVarianceContours), (*bigVarianceContours), CV_BGR2GRAY);

    double minVal, maxVal;
    cv::minMaxLoc((*bigVarianceContours), &minVal, &maxVal);
    (*bigVarianceContours).convertTo((*bigVarianceContours), CV_8UC1, 255.0 / (maxVal - minVal));
    cv::minMaxLoc((*bigVarianceContours), &minVal, &maxVal);

    // keep contours with the biggest variance, aka. the most white
    cv::threshold(
        (*bigVarianceContours), 
        (*bigVarianceContours), 
        Rgb::std_variance_threshold, 
        255, 
        CV_THRESH_BINARY);

    int morphologyKernel = Rgb::std_variance_morphology_close_size;
    cv::Mat structuringElement = 
      cv::getStructuringElement(
          cv::MORPH_ELLIPSE, 
          cv::Size(morphologyKernel, morphologyKernel));
    cv::morphologyEx((*bigVarianceContours), (*bigVarianceContours), cv::MORPH_CLOSE, structuringElement);
    morphologyKernel = Rgb::std_variance_morphology_open_size;
    structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morphologyKernel, morphologyKernel));
    cv::morphologyEx((*bigVarianceContours), (*bigVarianceContours), cv::MORPH_OPEN, structuringElement );
    // eliminate borders
    for(int i = 0; i < Rgb::border_thresh; i ++)
      for(int j = 0; j < (*bigVarianceContours).cols; j ++)
        (*bigVarianceContours).at<uchar>(i, j) = 0;
    for(int i = (*bigVarianceContours).rows - Rgb::border_thresh; i < (*bigVarianceContours).rows; i ++)
      for(int j = 0; j < (*bigVarianceContours).cols; j ++)
        (*bigVarianceContours).at<uchar>(i, j) = 0;
    for(int i = 0; i < (*bigVarianceContours).rows; i ++)
      for(int j = 0; j < Rgb::border_thresh; j ++)
        (*bigVarianceContours).at<uchar>(i, j) = 0;
    for(int i = 0; i < (*bigVarianceContours).rows; i ++)
      for(int j = (*bigVarianceContours).cols - Rgb::border_thresh; j < (*bigVarianceContours).cols; j ++)
        (*bigVarianceContours).at<uchar>(i, j) = 0;
#ifdef DEBUG_SHOW
    if (Rgb::show_std_variance_image)
    {
      Visualization::show("Std variance image", (*bigVarianceContours), 1);
    }
#endif
  }


  /**
    @brief The function called to extract contours that were not ignored by the initial filtering held in computeVarianceImage function.
    @param[in] bigVarianceContours [cv::Mat& bigVarianceContours] The variance image represented as edges. Remember that there were only edges left, which represent big variance.
    @param[in] contours [std::vector<std::vector<cv::Point>>*] The contours found.
    @return void
   **/
  void RgbProcessor::detectContours(const cv::Mat& bigVarianceContours, std::vector<std::vector<cv::Point> >* contours)
  {
    int erodeKernel = Rgb::contour_erode_kernel_size;
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeKernel, erodeKernel));
    cv::erode(bigVarianceContours, bigVarianceContours, structuringElement);
    cv::vector<cv::Vec4i> hierarchy;
    findContours(
        bigVarianceContours, 
        (*contours), 
        hierarchy, 
        CV_RETR_EXTERNAL, 
        CV_CHAIN_APPROX_SIMPLE, 
        cv::Point(0, 0));
  }


  /**
    @brief The function called to estimate center of mass for each contour found and bounding boxes.
    @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
    @param[in] mc [std::vector<cv::Point2f>*] Center of mass of each contour as x, y coordinates..
    @return void
   **/
  void RgbProcessor::getContourInfo(
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
    @param[in] image [cv::Mat&] The original image. Used to extract some features to evaluate contour similarity etc.
    @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
    @param[in] mc [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
    @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
    @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
    @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
    @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
    @return void
   **/
  void RgbProcessor::validateContours(
      const cv::Mat& image, 
      const std::vector<std::vector<cv::Point> >& contours, 
      std::vector<cv::Point2f>* mc, 
      std::vector<int>* contourHeight, 
      std::vector<int>* contourWidth, 
      std::vector<bool>* realContours, 
      const std::vector<cv::Rect>& boundRect)
  {
    for(int ci = 0; ci < contours.size(); ci ++)
    {
      if(Rgb::shape_validation)
        validateShape(image, contours, boundRect, ci, &(*realContours));
      if((*realContours)[ci])

        if((contours.size() > Rgb::lower_contour_number_to_test_huge 
              && cv::contourArea(contours[ci]) > Rgb::huge_contour_thresh))
        {
          (*realContours)[ci] = false;
          continue;
        }
        else if(cv::contourArea(contours[ci]) < Rgb::tiny_contour_thresh)
        {
          //if((*mc)[ci].x < Rgb::border_thresh 
          //    || (*mc)[ci].y < Rgb::border_thresh 
          //    || (image.cols - (*mc)[ci].x) < Rgb::border_thresh 
          //    || (image.rows - (*mc)[ci].y) < Rgb::border_thresh)
          //{
          (*realContours)[ci] = false;
          continue;
          //}
        }
        else
        {

          cv::Mat ROI = image(boundRect[ci]);
          cv::Scalar curAvg = cv::mean(ROI);
          for(int i = 0; i < contours.size(); i ++)
          {
            if(i != ci)
            {
              if((*realContours)[i] 
                  && (std::abs((*mc)[ci].x - (*mc)[i].x) < Rgb::neighbor_thresh) 
                  && (std::abs((*mc)[ci].y - (*mc)[i].y) < Rgb::neighbor_thresh))
              {
                int upperX;
                int upperY;
                int lowerX;
                int lowerY;
                if((*mc)[ci].x - (boundRect[ci].width / 2) > (*mc)[i].x - (boundRect[i].width / 2))
                  upperX = (*mc)[i].x - (boundRect[i].width / 2);
                else
                  upperX = (*mc)[ci].x - (boundRect[ci].width / 2);
                if((*mc)[ci].y - (boundRect[ci].height / 2) > (*mc)[i].y - (boundRect[i].height / 2))
                  upperY = (*mc)[i].y - (boundRect[i].height / 2);
                else
                  upperY = (*mc)[ci].y - (boundRect[ci].height / 2);
                if((*mc)[ci].x + (boundRect[ci].width / 2) > (*mc)[i].x + (boundRect[i].width / 2))
                  lowerX = (*mc)[ci].x + (boundRect[ci].width / 2);
                else
                  lowerX = (*mc)[i].x + (boundRect[i].width / 2);
                if((*mc)[ci].y + (boundRect[ci].height / 2) > (*mc)[i].y + (boundRect[i].height / 2))
                  lowerY = (*mc)[ci].y + (boundRect[ci].height / 2);
                else
                  lowerY = (*mc)[i].y + (boundRect[i].height / 2);
                cv::Mat ROI = image(boundRect[i]);
                cv::Scalar otherAvg = cv::mean(ROI);
                int homogRectWidth = std::abs(lowerX - upperX);
                if(homogRectWidth < Rgb::homog_rect_dims_thresh)
                  homogRectWidth = Rgb::homog_rect_dims_thresh;
                int homogRectHeight = abs(lowerY - upperY);
                if(homogRectHeight < Rgb::homog_rect_dims_thresh)
                  homogRectHeight = Rgb::homog_rect_dims_thresh;
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
                ROI = image(cv::Rect(upperX, upperY, homogRectWidth, homogRectHeight));
                HaralickFeaturesExtractor haralickFeaturesDetector_;
                haralickFeaturesDetector_.findHaralickFeatures(ROI);
                std::vector<double> haralickFeatures = haralickFeaturesDetector_.getFeatures();
                if((((std::abs(curAvg[0] - otherAvg[0]) < Rgb::neighbor_value_thresh) 
                        && haralickFeatures[0] > Rgb::homogenity_thresh)) 
                    || (((std::abs((*mc)[ci].x - (*mc)[i].x) < Rgb::neighbor_tiny_distance_thresh) 
                        && (std::abs((*mc)[ci].y - (*mc)[i].y) < Rgb::neighbor_tiny_distance_thresh)) 
                      && false))
                {
                  if(cv::contourArea(contours[i]) > cv::contourArea(contours[ci]))
                  {
                    (*mc)[i].x = 0.5 * (*mc)[i].x + 0.5 * (*mc)[ci].x;
                    (*mc)[i].y = 0.5 * (*mc)[i].y + 0.5 * (*mc)[ci].y;
                    (*contourHeight)[i] = 
                      (*contourHeight)[i] 
                      + (*contourHeight)[ci] 
                      + static_cast<int>(std::abs((*mc)[ci].y - (*mc)[i].y));
                    (*contourWidth)[i] = 
                      (*contourWidth)[i] 
                      + (*contourWidth)[ci] 
                      + static_cast<int>(std::abs((*mc)[ci].x - (*mc)[i].x));
                    (*realContours)[ci] = false;
                    continue;
                  }
                  else
                  {
                    (*mc)[ci].x = 0.5 * (*mc)[i].x + 0.5 * (*mc)[ci].x;
                    (*mc)[ci].y = 0.5 * (*mc)[i].y + 0.5 * (*mc)[ci].x;
                    (*contourHeight)[ci] = 
                      (*contourHeight)[ci] 
                      + (*contourHeight)[i] 
                      + static_cast<int>(std::abs((*mc)[ci].y - (*mc)[i].y));
                    (*contourWidth)[ci] = 
                      (*contourWidth)[ci] 
                      + (*contourWidth)[i] 
                      + static_cast<int>(std::abs((*mc)[ci].x - (*mc)[i].x));
                    (*realContours)[i] = false;
                  }
                }
              }
            }
          }

        }
    }
    for( int i = 0; i < contours.size(); i++ )
    {
      if((*realContours)[i])
        if((*contourWidth)[i] > Rgb::rect_diff_thresh * (*contourHeight)[i] 
            || (*contourHeight)[i] > Rgb::rect_diff_thresh * (*contourWidth)[i])
          (*realContours)[i] = false;
    }
  }


  void RgbProcessor::validateShape(
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
    int maxIntersectionsX = 0;
    int maxIntersectionsY = 0;

    // A vector that collects all the intersections numbers, or a sample of
    // them, from X direction, to check the standard deviation and their
    // mean
    std::vector<int> intersectionsX;

    // A vector that collects all the intersections numbers, or a sample of
    // them, from Y direction, to check the standard deviation and their
    // mean
    std::vector<int> intersectionsY;

    // A vector that collects all the pixels met inside a the ROI, by raycasting,
    // or a sample from the ray casts from X direction,
    //  to check the standard deviation and their mean
    std::vector<int> pixelsInsideX;

    // or a sample from the ray casts from Y direction,
    //  to check the standard deviation and their mean
    std::vector<int> pixelsInsideY;


    // Cost function consisting from mean and std_variance
    // of intersections, and internal pixels of the contours
    float shapeValidityFunction = 0;

    for(int row = boundRect[ci].y; row < (boundRect[ci].y + boundRect[ci].height); row ++)
    {
      bool isInside = false;
      bool isOutline = false;
      int intersectionCurrent = 0;
      int pixelsInsideCurrent = 0;
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
          // check if by leaving the white border an even or odd number
          // of boundaries have passed. In the case of odd number it is
          // most likely that we are at ROI's internal 
          if(intersectionCurrent % 2 != 0)
          {
            isInside = true;
            pixelsInsideCurrent++;
          }
          else
            isInside = false;
          isOutline = false;
        }
      }
      intersectionsX.push_back(intersectionCurrent);
      pixelsInsideX.push_back(pixelsInsideCurrent);
      if(intersectionCurrent > maxIntersectionsX)
        maxIntersectionsX = intersectionCurrent;
    }


    for(int col = boundRect[ci].x; col < (boundRect[ci].x + boundRect[ci].width); col ++)
    {
      bool isInside = false;
      bool isOutline = false;
      int intersectionCurrent = 0;
      int pixelsInsideCurrent = 0;
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
          // check if by leaving the white border an even or odd number
          // of boundaries have passed. In the case of odd number it is
          // most likely that we are at ROI's internal 
          if(intersectionCurrent % 2 != 0)
          {
            isInside = true;
            pixelsInsideCurrent++;
          }
          else
            isInside = false;
          isOutline = false;
        }
      }
      intersectionsY.push_back(intersectionCurrent);
      pixelsInsideY.push_back(pixelsInsideCurrent);
      if(intersectionCurrent > maxIntersectionsY)
        maxIntersectionsY = intersectionCurrent;
    }
    double sum = std::accumulate(intersectionsX.begin(), intersectionsX.end(), 0.0);
    double mean = sum / intersectionsX.size();
    // Fewer than two intersections alert for an open contour, so punish
    // them (if unclosed_contour_punishment parameter is selected over 1.0)
    // Take diferrence between mean and 2, because a normal- valid- contour will have
    // 2 intersections in avg
    if(mean >= 2)
      shapeValidityFunction += (Rgb::intersections_mean_cost / 2) * std::abs(mean - 2);
    else
    {
      shapeValidityFunction += 
        Rgb::unclosed_contour_punishment * (Rgb::intersections_mean_cost / 2) * std::abs(mean - 2);
    }

    std::vector<double> diff(intersectionsX.size());
    std::transform(intersectionsX.begin(), intersectionsX.end(), diff.begin(),
        std::bind2nd(std::minus<double>(), mean));
    double sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    float stdDevIntersections = std::sqrt(sqSum / intersectionsX.size());
    shapeValidityFunction += Rgb::intersections_stddev_cost * stdDevIntersections;

    sum = 0;
    sum = std::accumulate(intersectionsY.begin(), intersectionsY.end(), 0.0);
    mean = sum / intersectionsY.size();
    // Fewer than two intersections alert for an open contour, so punish
    // them (if unclosed_contour_punishment parameter is selected over 1.0)
    // Take diferrence between mean and 2, because a normal- valid- contour will have
    // 2 intersections in avg
    if(mean >= 2)
      shapeValidityFunction += (Rgb::intersections_mean_cost / 2) * std::abs(mean - 2);
    else
    {
      shapeValidityFunction += 
        Rgb::unclosed_contour_punishment * (Rgb::intersections_mean_cost / 2) * std::abs(mean - 2);
    }
    diff.clear();
    diff.resize(intersectionsY.size());
    std::transform(intersectionsY.begin(), intersectionsY.end(), diff.begin(),
        std::bind2nd(std::minus<double>(), mean));
    sqSum = 0;
    sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stdDevIntersections = std::sqrt(sqSum / intersectionsY.size());
    shapeValidityFunction += Rgb::intersections_stddev_cost * stdDevIntersections;

    sum = 0;
    sum = std::accumulate(pixelsInsideX.begin(), pixelsInsideX.end(), 0.0);
    mean = sum / pixelsInsideX.size();
    // take diferrence between mean and 2, because a normal contour will have
    // 2 intersections in avg
    shapeValidityFunction += 
      (Rgb::internal_pixels_2d_mean_cost / 2) * (1 - mean / std::pow(boundRect[ci].width, 2));
    //diff.clear();
    diff.clear();
    diff.resize(pixelsInsideX.size());
    std::transform(pixelsInsideX.begin(), pixelsInsideX.end(), diff.begin(),
        std::bind2nd(std::minus<double>(), mean));
    sqSum = 0;
    sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    float stdDevInternalPixelsX = std::sqrt(sqSum / pixelsInsideX.size());

    // Normalize 
    stdDevInternalPixelsX = stdDevInternalPixelsX / boundRect[ci].width; 
    shapeValidityFunction += (Rgb::internal_pixels_2d_stddev_cost / 2) * stdDevInternalPixelsX;

    sum = 0;
    sum = std::accumulate(pixelsInsideY.begin(), pixelsInsideY.end(), 0.0);
    mean = sum / pixelsInsideY.size();
    shapeValidityFunction += 
      (Rgb::internal_pixels_2d_mean_cost / 2) * (1 - mean / std::pow(boundRect[ci].height, 2));
    //diff.clear();
    diff.clear();
    diff.resize(pixelsInsideY.size());
    std::transform(pixelsInsideY.begin(), pixelsInsideY.end(), diff.begin(),
        std::bind2nd(std::minus<double>(), mean));
    sqSum = 0;
    sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    float stdDevInternalPixelsY = std::sqrt(sqSum / pixelsInsideY.size());

    // Normalize 
    stdDevInternalPixelsY = stdDevInternalPixelsY / boundRect[ci].height; 
    shapeValidityFunction += (Rgb::internal_pixels_2d_stddev_cost / 2) * stdDevInternalPixelsY;

    if(shapeValidityFunction > Rgb::shape_validity_thresh)
      (*realContours)[ci] = false;
    //if ((stdDevInternalPixelsX > Rgb::max_internal_pixels_variance_thresh
    //    || stdDevInternalPixelsY > Rgb::max_internal_pixels_variance_thresh)
    //  && stdDevIntersections > Rgb::max_intersections_variance_thresh)
    //{
    //(*realContours)[ci] = false;
    //} 

    //if ((boundRect[ci].height / avgPixelsInsideY >= 
    //      Rgb::one_direction_rectangle_contour_overlap_thresh 
    //      || boundRect[ci].width / avgPixelsInsideX >= 
    //      Rgb::one_direction_rectangle_contour_overlap_thresh)
    //    && (maxIntersectionsX > Rgb::max_intersections_thresh
    //      || maxIntersectionsY > Rgb::max_intersections_thresh))
    //  (*realContours)[ci] = false;
  }



  bool RgbProcessor::process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getImage().cols;
    output->frameHeight = input->getImage().rows;

#ifdef DEBUG_TIME
    Timer::start("rgbProcessor", "", true);
#endif

#ifdef DEBUG_SHOW
    if (Rgb::show_rgb_image)
    {
      Visualization::show("RGB image", input->getImage(), 1);
    }
#endif

    // Locate potential holes in the rgb image
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
    Timer::tick("rgbProcessor");
    Timer::printAllMeansTree();
#endif

    //if (output->pois.empty())
    //{
    //  return false;
    //}
    return true;
  }

}  // namespace pandora_vision

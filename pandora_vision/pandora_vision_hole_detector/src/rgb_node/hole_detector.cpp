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
 * Author: Despoina Paschalidou, Alexandros Philotheou
 *********************************************************************/

#include "rgb_node/hole_detector.h"

namespace pandora_vision
{
  /**
    @brief Class constructor
   **/
  HoleDetector::HoleDetector()
  {
    //! Calculate histogram according to a given set of images
    Histogram::getHistogram(&histogram_, Parameters::secondary_channel);

    ROS_INFO("[RGB node]: HoleDetector instance created");
  }



  /**
    @brief Class destructor
   **/
  HoleDetector::~HoleDetector()
  {
    ROS_INFO("[RGB node]: HoleDetector instance destroyed");
  }



  /**
    @brief Function that locates the position of potentional holes
    in the current frame.
    @param holeFrame [const cv::Mat&] current frame to be processed
    @return void
    */
  HolesConveyor HoleDetector::findHoles(const cv::Mat& holeFrame)
  {
    #ifdef DEBUG_TIME
    Timer::start("findHoles", "inputRgbImageCallback");
    #endif

    #ifdef SHOW_DEBUG_IMAGE
    std::string msg;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    #endif

    #ifdef SHOW_DEBUG_IMAGE
    cv::Mat initialRgbImage;
    holeFrame.copyTo(initialRgbImage);
    msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    msg += " : Initial RGB image";
    msgs.push_back(msg);
    imgs.push_back(initialRgbImage);
    #endif

    // Copy the input image to the segmentedHoleFrame one
    cv::Mat segmentedHoleFrame = cv::Mat(holeFrame.size(), CV_8UC3);
    holeFrame.copyTo(segmentedHoleFrame);


    bool posterize = false;

    // Segment the input image
    segmentation(holeFrame, posterize, &segmentedHoleFrame);

    Visualization::show("segmentedHoleFrame", segmentedHoleFrame, 1);

    // Convert the RGB segmented frame to grayscale
    cv::Mat segmentedHoleFrame8UC1 =
      cv::Mat::zeros(segmentedHoleFrame.size(), CV_8UC1);

    if (posterize)
    {
      // In order to find the edges of the segmented image,
      // first, turn it to grayscale
      cv::cvtColor(segmentedHoleFrame, segmentedHoleFrame8UC1, CV_BGR2GRAY);

      Visualization::show("segmentedHoleFrame8UC1", segmentedHoleFrame8UC1, 1);

      // Apply edge detection to the grayscale posterized input RGB image
      if (Parameters::edge_detection_method == 0)
      {
        EdgeDetection::applyCanny(segmentedHoleFrame8UC1,
          &segmentedHoleFrame8UC1);
      }
      else if (Parameters::edge_detection_method == 1)
      {
        EdgeDetection::applyScharr(segmentedHoleFrame8UC1,
          &segmentedHoleFrame8UC1);
      }
      else if (Parameters::edge_detection_method == 2)
      {
        EdgeDetection::applySobel(segmentedHoleFrame8UC1,
          &segmentedHoleFrame8UC1);
      }
      else if (Parameters::edge_detection_method == 3)
      {
        EdgeDetection::applyLaplacian(segmentedHoleFrame8UC1,
          &segmentedHoleFrame8UC1);
      }
      else if (Parameters::edge_detection_method == 4) // Mixed mode
      {
        if (Parameters::mixed_edges_toggle_switch == 1)
        {
          EdgeDetection::applyScharr(segmentedHoleFrame8UC1,
            &segmentedHoleFrame8UC1);
          Parameters::mixed_edges_toggle_switch = 2;
        }
        else if (Parameters::mixed_edges_toggle_switch == 2)
        {
          EdgeDetection::applySobel(segmentedHoleFrame8UC1,
            &segmentedHoleFrame8UC1);
          Parameters::mixed_edges_toggle_switch = 1;
        }
      }

      Visualization::show("segmentation->edges", segmentedHoleFrame8UC1, 1);
    }
    else
    {
      produceEdgesFromSegmentationThroughBackprojection(segmentedHoleFrame,
        &segmentedHoleFrame8UC1);
    }

    // Denoise the edges image
    EdgeDetection::denoiseEdges(&segmentedHoleFrame8UC1);

    #ifdef SHOW_DEBUG_IMAGE
    msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    msg += " : After denoising";
    msgs.push_back(msg);
    imgs.push_back(segmentedHoleFrame8UC1);
    #endif

    //! Find pixels in current frame where there is the same texture
    //! according to the given histogram and calculate
    std::vector<cv::KeyPoint> detectedkeyPoints;
    BlobDetection::detectBlobs(segmentedHoleFrame8UC1, &detectedkeyPoints);


    // The final vectors of keypoints, rectangles and blobs' outlines.
    HolesConveyor conveyor;

    HoleFilters::validateBlobs(
      &detectedkeyPoints,
      &segmentedHoleFrame8UC1,
      Parameters::bounding_box_detection_method,
      &conveyor);

    #ifdef SHOW_DEBUG_IMAGE
    msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    msg += STR(" : Blobs");
    msgs.push_back(msg);
    imgs.push_back(
      Visualization::showHoles(
        msg,
        initialRgbImage,
        -1,
        conveyor.keyPoints,
        conveyor.rectangles,
        std::vector<std::string>(),
        conveyor.outlines)
      );
    #endif

    #ifdef SHOW_DEBUG_IMAGE
    //Visualization::multipleShow("RGB node", imgs, msgs, 800, 1);
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("findHoles");
    #endif

    return conveyor;
  }



  /**
    @brief Fills an image with random colours per image segment
    @param[in,out] image [cv::Mat*] The image to be processed
    (see http://docs.opencv.org/modules/imgproc/doc/
    miscellaneous_transformations.html#floodfill)
    @return void
   **/
  void HoleDetector::floodFillPostprocess(cv::Mat* image)
  {
    if (image->type() != CV_8UC3)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("HoleDetector::floodFillPostprocess Inappropriate image type.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("floodFillPostprocess", "segmentation");
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
            cv::Scalar::all(Parameters::floodfill_lower_colour_difference),
            cv::Scalar::all(Parameters::floodfill_upper_colour_difference));
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("floodFillPostprocess");
    #endif
  }



  /**
    @brief This method takes as input a segmented RGB image and uses
    the backprojection of the input image based on the precalculated
    histogram histogram_ in order to identify whole regions whose
    histogram matches histogram_, although the backprojection image
    might be sparcely populated. After the identification of the regions
    of interest, this method extracts their edges and returns the image
    depicting them.
    @param[in] inImage [const cv::Mat&] The input segmented image
    @param[out] outImage [cv::Mat*] The output edges image, in CV_8UC1
    format
    @return void
   **/
  void HoleDetector::produceEdgesFromSegmentationThroughBackprojection
    (const cv::Mat& inImage, cv::Mat* outImage)
    {
      if (inImage.type() != CV_8UC3)
      {
        #ifdef DEBUG_SHOW
        ROS_ERROR("HoleDetector::\
          produceEdgesFromSegmentationThroughBackprojection: \
          Inappropriate image type.");
        #endif

        return;
      }

      #ifdef DEBUG_TIME
      Timer::start("produceEdgesFromSegmentationThroughBackprojection",
        "findHoles");
      #endif

      // Backprojection of the RGB inImage
      cv::Mat backprojectedFrame = cv::Mat::zeros(inImage.size(), CV_8UC1);

      // Get the backprojected image of the frame, based on the precalculated
      // histogram_ histogram
      Histogram::getBackprojection(inImage, histogram_,
        &backprojectedFrame, Parameters::secondary_channel);

      cv::threshold(backprojectedFrame, backprojectedFrame,
        40, 255, cv::THRESH_BINARY);

      Visualization::show("bp ", backprojectedFrame, 1);

      // The foreground image needed by the watershed algorithm
      cv::Mat foreground = cv::Mat::zeros(inImage.size(), CV_8UC1);

      // Copy the backprojection to the foreground image
      backprojectedFrame.copyTo(foreground);

      // Dilate
      Morphology::dilationRelative(&foreground,
        Parameters::watershed_foreground_dilation_factor);

      // All non-zero pixels have now a value of 255
      cv::threshold(foreground, foreground, 0, 255, cv::THRESH_BINARY);

      // Erode. The erosion factor should be greater
      // than the dilation factor used above
      Morphology::erosion(&foreground,
        Parameters::watershed_foreground_erosion_factor);

      Visualization::show("fg", foreground, 1);


      // The background image needed by the watershed algorithm
      cv::Mat background = cv::Mat::zeros(inImage.size(), CV_8UC1);

      // Copy the backprojection to the background image
      backprojectedFrame.copyTo(background);

      // Dilate
      Morphology::dilationRelative(&background,
        Parameters::watershed_background_dilation_factor);

      // All non-zero pixels have now a value of 255
      cv::threshold(background, background, 0, 255, cv::THRESH_BINARY);

      // All zero value pixels turn to white, all white to black
      cv::threshold(background, background, 0, 255, cv::THRESH_BINARY_INV);

      // Erode. The erosion factor should be greater
      // than the dilation factor used above. This erosion happens so that
      // the background pixels belong surely to the background
      Morphology::erosion(&background,
        Parameters::watershed_background_erosion_factor);

      // All zero value pixels' values are elevated to 128.
      // These belong neither to foreground, nor to background:
      // they are labeled as so-called "unknown"
      cv::threshold(background, background, 0, 128, cv::THRESH_BINARY);
      Visualization::show("bg", background, 1);

      // Create the markers image, needed by the watershed algorighm
      cv::Mat markers(inImage.size(), CV_8UC1, cv::Scalar(0));

      // The markers array is composed by the foreground, background and
      // unknown pixels
      markers = foreground + background;

      Visualization::show("markers", markers, 1);
      Visualization::show("outImage", *outImage, 1);

      // Convert the marker image of type CV_8UC1, to CV_32S
      cv::Mat markers32S;
      markers.convertTo(markers32S, CV_32S);

      // Watershed the product of the segmentation.
      // Each pixel p is transformed into
      // 255p + 255 before conversion
      cv::watershed(inImage, markers32S);

      // Convert the markers image to back to type CV_8UC1.
      // This image identifies the whole area that matches the histogram
      // histogram_ in the input image
      markers32S.convertTo(*outImage, CV_8U, 255, 255);

      // All zero value pixels turn to white, all white to black.
      // In essence, this is the edges of the area whose histogram
      // matches histogram_
      cv::threshold(*outImage, *outImage, 0, 255, cv::THRESH_BINARY_INV);

      Visualization::show("*outImage", *outImage, 1);

      #ifdef DEBUG_TIME
      Timer::tick("produceEdgesFromSegmentationThroughBackprojection");
      #endif
    }



  /**
    @brief Segments a RGB image
    @param[in] inImage [const cv::Mat&] The RGB image to be segmented
    @param[in] posterize [const bool&] Indicates the appliance of a
    random color to each segment
    @param[out] outImage [cv::Mat*] The posterized image
    @return void
   **/
  void HoleDetector::segmentation(const cv::Mat& inImage,
    const bool& posterize, cv::Mat* outImage)
  {
    if (inImage.type() != CV_8UC3)
    {
      #ifdef DEBUG_SHOW
      ROS_ERROR("HoleDetector::segmentation: Inappropriate image type.");
      #endif

      return;
    }

    #ifdef DEBUG_TIME
    Timer::start("segmentation", "findHoles");
    #endif

      // Blur the input image
      if (Parameters::segmentation_blur_method == 0)
      {
        // Segment the image
        cv::pyrMeanShiftFiltering(inImage, *outImage,
          Parameters::spatial_window_radius,
          Parameters::color_window_radius,
          Parameters::maximum_level_pyramid_segmentation);
      }
      else if (Parameters::segmentation_blur_method == 1)
      {
        for ( int i = 1; i < 15; i = i + 4 )
        {
          cv::medianBlur(inImage, *outImage, i);
        }
      }

      if (posterize)
      {
        // Fill the various segments with colour
        floodFillPostprocess(outImage);
      }

      #ifdef DEBUG_TIME
      Timer::tick("segmentation");
      #endif
  }


} // namespace pandora_vision

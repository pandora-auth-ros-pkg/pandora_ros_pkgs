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
* Author: Despoina Paschalidou, Vasilis Bosdelekidis
*********************************************************************/

#include "pandora_vision_datamatrix/datamatrix_detector.h"

namespace pandora_vision
{
namespace pandora_vision_datamatrix
{
  /**
   *@brief Constructor
  **/
  DatamatrixDetector::DatamatrixDetector()
  {
    img = NULL;
    dec = NULL;
    reg = NULL;
    msg = NULL;

    detected_datamatrix.reset( new DataMatrixPOI );
    detected_datamatrix->setContent("");
  }

  /**
    @brief Destructor
   */
  DatamatrixDetector::~DatamatrixDetector()
  {
    //!< Deallocate memory
    dmtxMessageDestroy(&msg);
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&img);
    dmtxRegionDestroy(&reg);
  }

  /**
    @brief Detects datamatrixes and stores them in a vector.
    @param image [cv::Mat] The image in which the QRs are detected
    @return void
   */
  std::vector<POIPtr> DatamatrixDetector::detect_datamatrix(cv::Mat image)
  {
    if (image.channels() < 3)
      cv::cvtColor(image, image, CV_GRAY2BGR);

    datamatrix_list.clear();

    img = NULL;
    dec = NULL;
    reg = NULL;
    msg = NULL;

    //!< creates and initializes a new DmtxImage structure using pixel
    //!< data provided  by  the calling application.
    img = dmtxImageCreate(image.data, image.cols, image.rows,
        DmtxPack24bppBGR);
    ROS_ASSERT(img != NULL);

    //!< creates and initializes a new DmtxDecode struct, which
    //!< designates the image to be scanned and initializes the scan
    //!< grid pattern.
    dec = dmtxDecodeCreate(img, 1);
    ROS_ASSERT(dec != NULL);

    //!< add msecs to timeout
    timeout = dmtxTimeAdd(dmtxTimeNow(), 1000);

    //!< searches every pixel location in a grid pattern looking
    //!< for potential barcode regions. A DmtxRegion is returned
    //!< whenever a potential barcode region is found, or if the final
    //!< pixel location has been scanned.
    reg = dmtxRegionFindNext(dec, &timeout);
    if (reg != NULL)
    {
      msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
      if (msg != NULL)
      {
        detected_datamatrix->getContent().assign((const char*) msg->output,
          msg->outputIdx);

        //!< Find datamatrixe's center exact position
        locate_datamatrix(image);
        datamatrix_list.push_back(detected_datamatrix);
      }
    }
    return datamatrix_list;
  }

  /**
    @brief Function that finds the position of datamatrixe's center
    @param image [cv::Mat] The image in which the QRs are detected
    @return void
   */
  void DatamatrixDetector::locate_datamatrix(cv::Mat image)
  {
    DmtxVector2 p00, p10, p11, p01;
    std::vector<cv::Point2f> datamatrixVector;

    p00.X = p00.Y = p01.X = p10.Y = 0.0;
    p01.Y = p10.X = p11.X = p11.Y = 1.0;

    dmtxMatrix3VMultiplyBy(&p00, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p10, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p11, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p01, reg->fit2raw);

    cv::Point2f corner1(p00.X, image.rows - p00.Y);
    cv::Point2f corner2(p10.X, image.rows - p10.Y);
    cv::Point2f corner3(p11.X, image.rows - p11.Y);
    cv::Point2f corner4(p01.X, image.rows - p01.Y);

    datamatrixVector.push_back(corner1);
    datamatrixVector.push_back(corner2);
    datamatrixVector.push_back(corner4);
    datamatrixVector.push_back(corner3);

    cv::RotatedRect calculatedRect;
    calculatedRect = minAreaRect(datamatrixVector);

    detected_datamatrix->setPoint(calculatedRect.center);

    if (visualizationFlag_)
      debug_show(image, datamatrixVector);
  }

  /**
    @brief Function that creates view for debugging purposes.
    @param image [cv::Mat] The image in which the datamatrixes are detected
    @param datamatrixVector [std::vector<cv::Point2f>] The vector of 4 corners
    of datamatrix image, according to which i draw lines for debug reasons
    @return debug_frcame [cv::Mat], frame with rotated rectangle
    and center of it
  */
  void DatamatrixDetector::debug_show(cv::Mat image, std::vector<cv::Point2f> datamatrixVector)
  {
    image.copyTo(debug_frame);
    cv::line(debug_frame, datamatrixVector.at(0), datamatrixVector.at(1), cv::Scalar(0, 255, 0), 3);
    cv::line(debug_frame, datamatrixVector.at(1), datamatrixVector.at(2), cv::Scalar(255, 0, 0), 3);
    cv::line(debug_frame, datamatrixVector.at(2), datamatrixVector.at(3), cv::Scalar(0, 0, 255), 3);
    cv::line(debug_frame, datamatrixVector.at(3), datamatrixVector.at(0), cv::Scalar(255, 255, 0), 3);

    cv::RotatedRect calculatedRect;
    calculatedRect = minAreaRect(datamatrixVector);
    for (int i = 0; i < 4; i++)
    {
      line(debug_frame, datamatrixVector[i], datamatrixVector[(i + 1) % 4], cv::Scalar(255, 0, 0));
    }
    cv::circle(debug_frame, calculatedRect.center, 4, cv::Scalar(0, 0, 255), 8, 8);
    ROS_INFO_STREAM("Angle given by minAreaRect:" << calculatedRect.angle);
    cvtColor(debug_frame, debug_frame, CV_BGR2GRAY);

    cv::imshow("Debug DataMatrix Image", debug_frame);
    cv::waitKey(5);
  }
}  // namespace pandora_vision_datamatrix
}  // namespace pandora_vision

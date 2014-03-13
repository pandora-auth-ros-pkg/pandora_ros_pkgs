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
* Authors:  Tsakalis Vasilis, Despoina Paschalidou
*********************************************************************/

#ifndef PANDORA_VISION_HAZMAT_INCLUDE_PANDORA_VISION_HAZMAT_DEFINES_H_
#define PANDORA_VISION_HAZMAT_INCLUDE_PANDORA_VISION_HAZMAT_DEFINES_H_

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <dirent.h>
#include <time.h>
#include <cstring>
#include <errno.h>
#include <limits.h>
#include <vector>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ros/package.h>
#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "state_client.h"
#include "vision_communications/HazmatAlertsVectorMsg.h"

//the maximum number of keypoint NN candidates to check during BBF search
#define KDTREE_BBF_MAX_NN_CHKS 200
//threshold on squared ratio of distances between NN and 2nd NN
#define NN_SQ_DIST_RATIO_THR 0.49

//!< Defines from hazmat_detection
#define HFOV  61.14           //68 hfov in degrees : giwrgos 61.142
#define VFOV  48              //50 vfov in degrees : giwrgos 47.79
#define DEFAULT_HEIGHT  480   //default frame height
#define DEFAULT_WIDTH   640   //default frame width

/* absolute value */
#ifndef ABS
#define ABS(x) ( ( x < 0 )? -x : x )
#endif

/* RANSAC error tolerance in pixels */
#define RANSAC_ERR_TOL 3

/** pessimistic estimate of fraction of inlers for RANSAC */
#define RANSAC_INLIER_FRAC_EST 0.25

/** estimate of the probability that a correspondence supports a bad model */
#define RANSAC_PROB_BAD_SUPP 0.10

/** default number of sampled intervals per octave */
#define SIFT_INTVLS 3

/** default sigma for initial gaussian smoothing */
#define SIFT_SIGMA 1.6

/** default threshold on keypoint contrast |D(x)| */
#define SIFT_CONTR_THR 0.04

/** default threshold on keypoint ratio of principle curvatures */
#define SIFT_CURV_THR 10

/** double image size before pyramid construction? */
#define SIFT_IMG_DBL 1

/** default width of descriptor histogram array */
#define SIFT_DESCR_WIDTH 4

/** default number of bins per histogram in descriptor array */
#define SIFT_DESCR_HIST_BINS 8

/* assumed gaussian blur for input image */
#define SIFT_INIT_SIGMA 0.5

/* width of border in which to ignore keypoints */
#define SIFT_IMG_BORDER 5

/* maximum steps of keypoint interpolation before failure */
#define SIFT_MAX_INTERP_STEPS 5

/* default number of bins in histogram for orientation assignment */
#define SIFT_ORI_HIST_BINS 36

/* determines gaussian sigma for orientation assignment */
#define SIFT_ORI_SIG_FCTR 1.5

/* determines the radius of the region used in orientation assignment */
#define SIFT_ORI_RADIUS 3.0 * SIFT_ORI_SIG_FCTR

/* number of passes of orientation histogram smoothing */
#define SIFT_ORI_SMOOTH_PASSES 2

/* orientation magnitude relative to max that results in new feature */
#define SIFT_ORI_PEAK_RATIO 0.8

/* determines the size of a single descriptor orientation histogram */
#define SIFT_DESCR_SCL_FCTR 3.0

/* threshold on magnitude of elements of descriptor vector */
#define SIFT_DESCR_MAG_THR 0.2

/* factor used to convert floating-point descriptor to unsigned char */
#define SIFT_INT_DESCR_FCTR 512.0

/* initial # of priority queue elements for which to allocate space */
#define MINPQ_INIT_NALLOCD 512

/* colors in which to display different feature types */
#define FEATURE_OXFD_COLOR CV_RGB(255, 255, 0)
#define FEATURE_LOWE_COLOR CV_RGB(255, 0, 255)

/** max feature descriptor length */
#define FEATURE_MAX_D 128

#endif  // PANDORA_VISION_HAZMAT_INCLUDE_PANDORA_VISION_HAZMAT_DEFINES_H_

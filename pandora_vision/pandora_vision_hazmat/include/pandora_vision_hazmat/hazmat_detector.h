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
 
#ifndef PANDORA_VISION_HAZMAT_HAZMAT_DETECTOR_H
#define PANDORA_VISION_HAZMAT_HAZMAT_DETECTOR_H

#include "pandora_vision_hazmat/sift.h"
#include "pandora_vision_hazmat/imgfeatures.h"
#include "pandora_vision_hazmat/kdtree.h"
#include "pandora_vision_hazmat/utils.h"
#include "pandora_vision_hazmat/xform.h"
#include "pandora_vision_hazmat/hazmat_parameters.h"

namespace pandora_vision{
  /**
    @struct HazmatEpsilon
    @brief The struct in which the results are stored
  **/ 
  struct HazmatEpsilon
  {
    int pattern_num;
    float x, y;
    int m;
    float MO;
    int votes;
    CvMat* H;
  };
  
  /**
    @class HazmatEpsilonDetector
    @brief The hazmat epsilon detection class
  **/
  class HazmatEpsilonDetector
  {
    private:
    
    /// Path to subfolder patterns in package pandora_vision_hazmat
    std::string pattern_index_path;
    
    /// Path to package pandora_vision_hazmat
    std::string package_path;
    
    /// Number of hazmats loaded in hard disk
    int nPatterns;
    
    int* nFeats;
    
    int votes;
    
    float votingThreshold;
    
    float scale;
    
    float area;
    
    float minAreaThreshold;
    
    float maxAreaThreshold;
    
    float MOThreshold;
    
    struct feature** feats;
    
    struct kd_node** trees;
    
    int sideLength;
    
    int colorVariance;
    
    int featureThreshold;
    
    int frameNum;
    
    int rows;
    int cols;
  public:
    /// Constructor
    explicit HazmatEpsilonDetector(std::string _package_path);
    
    /**
      @brief Default constructor
      @return void
    **/
    HazmatEpsilonDetector(void);
      
    /**
      @brief Default destructor
      @return void
    **/
    ~HazmatEpsilonDetector(void);
    
    ///Shows how many patterns were found in current screenshot  
    int patterns_found_counter;
    
    float** minUV;
    float**maxUV; 
    
    CvPoint2D64f** upperPoints;
    CvPoint2D64f** lowerPoints;
  
    std::vector<CvPoint2D64f> center_of;
    
    /**
     @brief Calculates histogramm for the given hazmat sign
     @return void
    */ 
    void calcHistograms();
    
    /**
     @brief Initializes hazmat detector. Loads hazmats from
     hard dik into memory
     @return void
    */ 
    void initDetector(); 
    
    /**
      @brief Reads contents from file "contents" and stores into memory \
      the processed hazmats
      @return void
    */
    void preprocessHazmat(); 
    
    /**
      @brief Sets the hazmat parameters
      @param clrVariance [int]
      @param votingThr [float]
      @param minAreaThr [float]
      @param maxAreaThr [float]
      @param sideLgth [int]
      @param featThr [int]
      @param MOThr [float]
      @return void
    **/
    void setHazmatParameters(int clrVariance, float votingThr,
        float minAreaThr, float maxAreaThr, int sideLgth, int featThr,
        float MOThr);
        
    /**
      @brief The core of hazmat detector
      @param img [cv::Mat] Current frame to be processed
      @return std::vector<HazmatEpsilon> of found hazmats in current frame
    */ 
    std::vector<HazmatEpsilon> detectHazmat(cv::Mat hazmatFrame); 
    
    /**
      @brief Finds a specific feature (?)
      @param m [int *]
      @param n [int]
      @param testNum [int]
      @param kd_root [struct kd_node *]
      @return void
    **/
    std::vector <int> findFeature(int *m, int n, int testNum,
        struct kd_node* kd_root);
    
    /**
      @brief Calculates the area of a rectangle from its four corners
      @param pt1 [CvPoint2D64f]
      @param pt2 [CvPoint2D64f]
      @param pt3 [CvPoint2D64f]
      @param pt4 [CvPoint2D64f]
      @return float : The area
    **/        
    float calculateRectangleArea(CvPoint2D64f pt1, CvPoint2D64f pt2, 
      CvPoint2D64f pt3, CvPoint2D64f pt4); 
    
    /**
      @brief Calculates the area of a pattern in an image 
      @param H [CvMat*]
      @param pattern_image [cv::Mat]
      @return void
    **/
    void calculateArea(CvMat* H, cv::Mat pattern_image);
    
    /**
      @brief (?)
      @param SAD [float*]
      @param SAD2 [float*]
      @param img [IplImage*]
      @param H [CvMat*]
      @param _pattern_image [cv::Mat]
      @param n [int]
      @return CvPoint2D64f
    **/
    CvPoint2D64f  defineVariance(float* SAD, float* SAD2, 
      IplImage* img, CvMat* H, cv::Mat _pattern_image, int n);  
    
    cv::MatND patternHistog;
    
    /**
      @brief (?)
      @return void
    **/ 
    void calcMinMax();
  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_HAZMAT_HAZMAT_DETECTOR_H

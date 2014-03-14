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

#include "pandora_vision_hazmat/hazmat_detector.h"

namespace pandora_vision
{

  /**
  @brief Default constructor
  @return void
  **/
  HazmatEpsilonDetector::HazmatEpsilonDetector(std::string package_path): 
    rows(480),
    cols(640)
  {
    param_path_ = package_path;
    setParameters();
    initDetector();
  }

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
  void HazmatEpsilonDetector::setHazmatParameters(
    int clrVariance,
    float votingThr,
    float minAreaThr,
    float maxAreaThr,
    int sideLgth,
    int featThr,
    float MOThr)
  {
    colorVariance_ = clrVariance;
    votingThreshold_ = votingThr;
    minAreaThreshold_ = minAreaThr;
    maxAreaThreshold_ = maxAreaThr;
    featureThreshold_ = featThr;
    MOThreshold_ = MOThr;
    sideLength_ = sideLgth;

    calcMinMax();
  }

  /**
  @brief Default destructor
  @return void
  **/
  HazmatEpsilonDetector::~HazmatEpsilonDetector()
  {
    delete[] nFeats_;
    delete[] minUV;
    delete[] maxUV;
  }

  /**
  @brief (?)
  @return void
  **/
  void HazmatEpsilonDetector::calcMinMax(void)
  {
    minUV = new float* [nPatterns_];
    maxUV = new float* [nPatterns_];
    cv::Mat image;
    
    for (int i = 0 ; i < nPatterns_ ; i++)
    {
      minUV[i] = new float [2];
      maxUV[i] = new float [2];
    }
    
    for (int i = 0 ; i < nPatterns_ ; i++)
    {
      char temp_name[50];
      snprintf(temp_name, sizeof(temp_name), "/patterns/enter%d.png", i + 1);
      std::string name = param_path_ + temp_name;
      
      image = cv::imread(name.c_str(), 1);
      cvtColor(image, image, CV_BGR2YCrCb);
      int height, width, step, channels;
      height    = image.size().height;
      width     = image.size().width;
      step      = image.step;
      channels  = image.channels();

      for (int j = 0 ; j < sideLength_ ; j++)
      {
        for (int k = 0 ; k < sideLength_ ; k++)
        {
          if( (j == 0) && (k == 0) )
          {
            minUV[i][0] = (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength_ / 2) + j) * step + 
                ((width / 2) - (sideLength_ / 2) + k) * channels + 1];
            minUV[i][1] = (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength_ / 2) + j) * step + 
                ((width / 2) - (sideLength_ / 2) + k) * channels + 2];
            maxUV[i][0] = (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength_ / 2) + j) * step + 
                ((width / 2) - (sideLength_ / 2) + k) * channels + 1];
            maxUV[i][1] = (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength_ / 2) + j) * step + 
                ((width / 2) - (sideLength_ / 2) + k) * channels + 2];
          }
          else
          {
            if(minUV[i][0] > (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength_ / 2) + j) * step + 
                ((width / 2) - (sideLength_ / 2) + k) * channels + 1])
            {
              minUV[i][0] = (reinterpret_cast<uchar *>(image.data))
                [((height / 2) - (sideLength_ / 2) + j) * step + 
                  ((width / 2) - (sideLength_ / 2) + k) * channels + 1];
            }
            
            if (minUV[i][1] > (reinterpret_cast<uchar *>(image.data))[
              ((height / 2) - (sideLength_ / 2) + j) * step +
                ((width / 2) - (sideLength_ / 2) + k) * channels + 2])
            {
              minUV[i][1] = (reinterpret_cast<uchar *>(image.data))[
                ((height / 2) - (sideLength_ / 2) + j) * step +
                  ((width / 2) - (sideLength_ / 2) + k) * channels + 2];
            }
            
            if(maxUV[i][0] < (reinterpret_cast<uchar *>(image.data))[
              ((height / 2) - (sideLength_ / 2) + j) * step +
                ((width / 2) - (sideLength_ / 2) + k) * channels + 1])
            {
              maxUV[i][0] = (reinterpret_cast<uchar *>(image.data))[
                ((height / 2) - (sideLength_ / 2) + j) * step +
                  ((width / 2) - (sideLength_ / 2) + k) * channels + 1];
            }
            
            if(maxUV[i][1] < (reinterpret_cast<uchar *>(image.data))[
              ((height / 2) - (sideLength_ / 2) + j) * step +
                ((width / 2) - (sideLength_ / 2) + k) * channels + 2])
            {
              maxUV[i][1] = (reinterpret_cast<uchar *>(image.data))[
                ((height / 2) - (sideLength_ / 2) + j) * step +
                  ((width / 2) - (sideLength_ / 2) + k) * channels + 2];
            }
          }
        }
      }
    }
  }

  /**
  @brief Calculates histograms for given hazmat signs
  @return void
  **/
  void HazmatEpsilonDetector::calcHistograms(void)
  {
      cv::Mat* patterns = new cv::Mat[nPatterns_];
      cv::Mat image;
      char temp_name[50];
      for(int i = 0 ; i < nPatterns_ ; i++)
      {
        snprintf(temp_name, sizeof(temp_name), "/patterns/enter%d.png", i + 1);
        std::string name = param_path_ + temp_name;
        image = cv::imread(name.c_str(), 1);
        cvtColor(image, image, CV_BGR2HSV);
        patterns[i] = image;
      }
      // Quantize the hue to 30 levels
      // and the saturation to 32 levels
      int hbins = 30;
      int histSize[] = {hbins};
      // hue varies from 0 to 179, see cvtColor
      float hranges[] = { 0, 180 };
      const float* ranges[] = { hranges};
      // we compute the histogram from the 0-th and 1-st channels
      int channels[] = { 0 };
      calcHist(
        patterns,
        nPatterns_, 
        channels, 
        cv::Mat(),
        patternHistog, 
        1, 
        histSize, 
        ranges,
        true, 
        false );
        
      delete[] patterns;
  }

  /**
  @brief Initialize class parameters. Can be tweaked to exchange \
  quality for speed.
  @return void
  **/
  void HazmatEpsilonDetector::setParameters(void)
  {
    // Set parameters
    char temp_name[50];
    snprintf(temp_name, sizeof(temp_name), "/contents");
    patternIndexPath_ = param_path_ + temp_name;
    
  } 

  /**
  @brief Calculates the area of a rectangle from its four corners
  @param pt1 [CvPoint2D64f]
  @param pt2 [CvPoint2D64f]
  @param pt3 [CvPoint2D64f]
  @param pt4 [CvPoint2D64f]
  @return float : The area
  **/
  float HazmatEpsilonDetector::calculateRectangleArea(
    CvPoint2D64f pt1,
    CvPoint2D64f pt2, 
    CvPoint2D64f pt3,
    CvPoint2D64f pt4)
  {
    float sideA, sideB, sideC; 
    
    sideA = sqrt(((pt1.x - pt2.x) * (pt1.x - pt2.x)) + 
      ((pt1.y - pt2.y) * (pt1.y - pt2.y)));
    
    sideB = sqrt(((pt2.x - pt3.x) * (pt2.x - pt3.x)) + 
      ((pt2.y - pt3.y) * (pt2.y - pt3.y)));
    
    sideC = sqrt(((pt1.x - pt3.x) * (pt1.x - pt3.x)) + 
      ((pt1.y - pt3.y) * (pt1.y - pt3.y)));
    
    float tempValue, triangleArea;
    
    tempValue = (sideA + sideB + sideC) / 2.0;
    triangleArea = sqrt( tempValue * (tempValue - sideA) * (tempValue - sideB) 
      * (tempValue - sideC) );
    
    float triangleArea2;
    
    sideA = sqrt(((pt1.x - pt3.x) * (pt1.x - pt3.x)) 
      + ((pt1.y - pt3.y) * (pt1.y - pt3.y)));
    
    sideB = sqrt(((pt1.x - pt4.x) * (pt1.x - pt4.x)) 
      + ((pt1.y - pt4.y) * (pt1.y - pt4.y)));
    
    sideC = sqrt(((pt3.x - pt4.x) * (pt3.x - pt4.x)) 
      + ((pt3.y - pt4.y) * (pt3.y - pt4.y)));
    
    tempValue = (sideA + sideB + sideC) / 2.0;
    
    triangleArea2 = sqrt(tempValue * (tempValue - sideA) * (tempValue - sideB) 
      * (tempValue - sideC));
    
    float rectangleArea = triangleArea + triangleArea2;
    
    return rectangleArea;
  }

  /**
   * @brief Draws detected hazmat for debuging purposes
   * @param img
   * @param target
   * @param H
   */
  static void draw_xform(cv::Mat* img, cv::Mat* target, CvMat* H )
  {
    CvPoint2D64f xc[4], 
      c[4] = { 
        { 0, 0 }, 
        { target->size().width - 1, 0 }, 
        { target->size().width - 1, target->size().height - 1 }, 
        { 0, target->size().height - 1 } };
    
    for(int i = 0 ; i < 4 ; i++ )
    {
      xc[i] = persp_xform_pt( c[i], H );
    }
    for(int i = 0 ; i < 4 ; i++ )
    {
      cv::line( 
        *img, 
        cv::Point( xc[i].x, xc[i].y ), 
        cv::Point( xc[(i + 1) % 4].x, xc[(i + 1) % 4].y ), 
        CV_RGB( 0, 255, 0 ), 10, CV_AA, 0 );
    }
  }

  /**
  @brief Initialize hazmat detector. Loads hazmats from hard disk \
  into memory.
  @return void
  **/
  void HazmatEpsilonDetector::initDetector(void)
  {
    // Read number of patterns

    FILE* contents = fopen(patternIndexPath_.c_str(), "r");
    
    if (contents == NULL)
    { 
      printf("Can't open contents file");
    }
       
    char imgName[50];
    char featName[50];
    int n;
    
    nPatterns_ = 0;
    
    while(true)
    {
      fgets(imgName, 49, contents);

      if(imgName[0] == '\n') 
      {
        break;
      }
      if (feof(contents) != 0) 
      {
        break;
      }

      nPatterns_++;
    }
    
    //Sets the position indicator associated with stream to the beginning \
    of the file.
    rewind(contents);
    
    // Load features of patterns in memory
    feats_ = new feature* [nPatterns_];
    
    nFeats_ = new int[nPatterns_];
    
    for(n = 0 ; n < nPatterns_ ; n++)
    {
      fgets(imgName, 49, contents);
      imgName[strlen(imgName) - 1] = '\0';
      snprintf(featName, sizeof(featName), "/%s.sift", imgName);

      std::string feature_name;
      feature_name = param_path_+featName;

      //Returns the number of features imported from every pattern
      nFeats_[n] = import_features(const_cast<char*>(feature_name.c_str())
        , FEATURE_LOWE, &feats_[n]);
    } 
    fclose(contents);
  }

  /**
  @brief Reads contents from file "contents" and stores into memory \
  the processed hazmats
  @return void
  **/
  void HazmatEpsilonDetector::preprocessHazmat(void)
  {
    // Read table of contents
    FILE* contents = fopen(patternIndexPath_.c_str(), "r");
    char imgName[50], featName[50];
    while( true )
    {
      // Get next pattern
      fgets(imgName, 49, contents);
      imgName[strlen(imgName) - 1] = '\0';
      if( strlen(imgName) == 0)
      {
        break;
      }
      
      printf("Processing hazmat \"%s\"\n", imgName);
      struct feature* feat;

      // Compute and store features
      IplImage* img = cvLoadImage(imgName, 1);
      
      if(!img) 
      {
        std::cout << "No image!\n" << std::endl;
      }
      
      int n = sift_features(img, &feat);
      snprintf(featName, sizeof(featName), "%s.sift", imgName);
      export_features(featName, feat, n);

      // Free memory
      cvReleaseImage(&img);
      delete feat;
    }
    fclose(contents);
  }

  /**
  @brief Finds a specific feature (?)
  @param m [int *]
  @param n [int]
  @param testNum [int]
  @param kd_root [struct kd_node *]
  @return void
  **/
  std::vector <int> HazmatEpsilonDetector::findFeature(
    int* m,
    int n,
    int testNum,
    struct kd_node* kd_root)
  {
    struct feature** nbrs;   
    int i;
    double d0, d1;
    std::vector <int> tempvector;
    for(i = 0 ; i < testNum ; i++)
    {
      int k = kdtree_bbf_knn( kd_root, feats_[n] + i, 2, &nbrs, 
        KDTREE_BBF_MAX_NN_CHKS );
      
      if( k == 2 )
      {
        d0 = descr_dist_sq( feats_[n] + i, nbrs[0] );
        d1 = descr_dist_sq( feats_[n] + i, nbrs[1] );
        if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
        {
          feats_[n][i].fwd_match = nbrs[0];
          tempvector.push_back(i);
          m[0]++;
        }
      }
      free(nbrs);
    }
    return tempvector;
  } 

  /**
  @brief Calculates the area of a pattern in an image (?)
  @param H [CvMat*]
  @param pattern_image [cv::Mat]
  @return void
  **/
  void  HazmatEpsilonDetector::calculateArea( CvMat* H, cv::Mat pattern_image)
  {
    CvPoint2D64f point1 = {0, 0};
    CvPoint2D64f point2 = {0, pattern_image.size().height};
    CvPoint2D64f point3 = {pattern_image.size().width, 0};
    CvPoint2D64f point4 = 
      {pattern_image.size().width, pattern_image.size().height};
    
    //find the transformation of the 4 corners of the initial image
    point1 = persp_xform_pt( point1, H );
    point2 = persp_xform_pt( point2, H );
    point3 = persp_xform_pt( point3, H );
    point4 = persp_xform_pt( point4, H );
    
    //calculate the final area
    area_ = calculateRectangleArea(point1, point2, point3, point4);
  }

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
  CvPoint2D64f HazmatEpsilonDetector::defineVariance(
    float* SAD,
    float* SAD2,
    IplImage* img,
    CvMat* H,
    cv::Mat _pattern_image,
    int n)  
  { 
    IplImage* pattern_image = 
      cvCreateImage( cvSize(cols, rows), IPL_DEPTH_8U, 3 );
      
    IplImage* temp = new IplImage(_pattern_image);
    pattern_image = cvCloneImage(temp);
   
    IplImage* _xformed = cvCreateImage( cvGetSize( img ), IPL_DEPTH_8U, 3 );
    cvWarpPerspective( pattern_image, _xformed, H, 
      CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cv::Scalar( 0, 0, 0 ) );
    
    cv::Mat xformed(_xformed, false);
   
    cv::Mat color_image = cv::Mat( cv::Size(cols, rows), CV_8U, 3 );
    xformed.copyTo(color_image);
    cvtColor(color_image, color_image, CV_BGR2YCrCb);
    cv::Mat _img(img, false);
    cv::Mat img_ycrcb = cv::Mat( cvSize(cols, rows), CV_8U, 3 );
    
    _img.copyTo(img_ycrcb);
    cvtColor(img_ycrcb, img_ycrcb, CV_BGR2YCrCb);

    int height, width, step, channels;
    int height2, width2, step2, channels2;
    
    height    = color_image.size().height;
    width     = color_image.size().width;
    step      = color_image.step;
    channels  = color_image.channels();
    height2    = img_ycrcb.size().height;
    width2     = img_ycrcb.size().width;
    step2      = img_ycrcb.step;
    channels2  = img_ycrcb.channels();
    
    CvPoint2D64f point = {
      (_pattern_image.size().width) / 2,
      (_pattern_image.size().height) / 2
    };
    
    point = persp_xform_pt( point, H );
    votes_ = 0;
              
    //start voting
    for (int counter = 0; counter < sideLength_ ; counter++)
    {
      for (int counter2 = 0; counter2 < sideLength_ ; counter2++)
      {
        if (( (point.y - (sideLength_ / 2) + counter ) >= 0) && 
          ( (point.y - (sideLength_ / 2) + counter ) < rows ))
        {
          if (( (point.x - (sideLength_/2) +counter2 ) >= 0) && 
            ( (point.x - (sideLength_/2) +counter2 ) < cols ))
          {
            bool test1 = 
              ( (reinterpret_cast<uchar*>(img_ycrcb.data)
                [ ( static_cast<int>(point.y) - sideLength_ / 2 + counter) * 
                    step + 
                  ( static_cast<int>(point.x) - sideLength_ / 2 + counter2) * 
                    channels + 1 ] 
                  >= 
                  (minUV[n][0] - 333333333333333333) )
              && 
              ( (reinterpret_cast<uchar*>(img_ycrcb.data)
                [ ( static_cast<int>(point.y) - sideLength_ / 2 + counter) * 
                    step + 
                  ( static_cast<int>(point.x) - sideLength_ / 2 + counter2) * 
                    channels + 1] 
                  <= 
                  (maxUV[n][0] + colorVariance_))));
                  
            if(test1 == true)
            {
              bool test2 = 
                ( (reinterpret_cast<uchar*>(img_ycrcb.data))
                  [ ( static_cast<int>(point.y) - sideLength_ / 2 + counter) * 
                      step + 
                    ( static_cast<int>(point.x) - sideLength_ / 2 + counter2) * 
                      channels + 2]
                    >= 
                    (minUV[n][1] - colorVariance_) ) 
                && 
                ( (reinterpret_cast<uchar*>(img_ycrcb.data))
                  [ ( static_cast<int>(point.y) - sideLength_ / 2 + counter) * 
                      step + 
                    ( static_cast<int>(point.x) - sideLength_ / 2 + counter2) * 
                      channels + 2]
                    <= 
                    (maxUV[n][1] + colorVariance_) );
              
              if(test2 == true)
              {
                votes_++;
              }
            }
            //find the absolute variance between the initial and the final image
            
            *SAD = *SAD + fabsf( 
              (reinterpret_cast<uchar*>(img_ycrcb.data))[
                ( static_cast<int>(point.y) - sideLength_ / 2 + counter) * 
                  step2 + 
                ( static_cast<int>(point.x) - sideLength_ / 2 + counter2) * 
                  channels2 + 1] - 
              (reinterpret_cast<uchar*>(color_image.data))[
                ( static_cast<int>(point.y) - sideLength_ / 2 + counter) * 
                  step + 
                ( static_cast<int>(point.x) - sideLength_ / 2 + counter2) * 
                  channels + 1] );
                 
            *SAD2 = *SAD2 + fabsf( 
              (reinterpret_cast<uchar*>(img_ycrcb.data))[
                ( static_cast<int>(point.y) - sideLength_ / 2 + counter) * 
                  step2 + 
                ( static_cast<int>(point.x) - sideLength_ / 2 + counter2) * 
                  channels2 + 2] - 
              (reinterpret_cast<uchar*>(color_image.data))[
                ( static_cast<int>(point.y) - sideLength_ / 2 + counter) * 
                  step + 
                ( static_cast<int>(point.x) - sideLength_ / 2 + counter2) * 
                  channels + 2]); 
          }
        }
      }
    }
    cvReleaseImage(&pattern_image);
    cvReleaseImage(&_xformed);
    delete temp;
    return point;
  }

  /**
  @brief The core of the hazmat detector. Detects hazmats in the screenshot
  @param hazmatFrame [cv::Mat]
  @return std::vector<HazmatEpsilon>
  **/
  std::vector<HazmatEpsilon> HazmatEpsilonDetector::detectHazmat(
    cv::Mat hazmatFrame)
  {
    std::vector<HazmatEpsilon> result;
    
    IplImage* img = cvCreateImage( cv::Size(cols, rows), IPL_DEPTH_8U, 3 );
    IplImage* temp = new IplImage(hazmatFrame);
    img = cvCloneImage(temp);
  
    initDetector();
    
    // SIFT process of screenshot
    struct kd_node* kd_root;
    
    if (!img)
    {
      ROS_ERROR("Could not load image");
    }
    
    struct feature* featShot;

    // Finds SIFT features in an image.
    int nShot = sift_features(img, &featShot);
    if (nShot > 0)
    {
      kd_root = kdtree_build( featShot, nShot );
      
      // Search screenshot for each pattern
      int n, i;
      int m = 0;
      int max = -1, pat;
      for(n = 0 ; n < nPatterns_ ; n++)
      {
        // Find nearest neighboor for each feature in screenshot
        
        int testNum = nFeats_[n];
        std::vector <int> tempvector;
        tempvector = findFeature(&m, n, testNum, kd_root);
        // Run RANSAC to find out a transformation that transforms patterns \
        into screenshot
        if( m > featureThreshold_ )
        {
          CvMat* H;
          H = ransac_xform( 
            feats_[n], 
            testNum, 
            FEATURE_FWD_MATCH, 
            lsq_homog, 
            4, 
            0.01, 
            homog_xfer_err,
            3.0, 
            NULL, 
            NULL 
          );
          if( H )
          {
            char temp_name[50];
            snprintf(temp_name, sizeof(temp_name), 
              "/patterns/enter%d.png", n + 1);
            std::string name;
            name = param_path_ + temp_name;
          
            cv::Mat pattern_image;
            pattern_image = cv::imread(name.c_str(), CV_LOAD_IMAGE_COLOR);
            
            if (!pattern_image.data)
            {
              ROS_ERROR("could not load pattern image");
            }

            calculateArea(H, pattern_image);
            if (area_ >= minAreaThreshold_ && area_ <= maxAreaThreshold_)
            {
              float SAD = 0;
              float SAD2 = 0;
              CvPoint2D64f point;
              point = defineVariance(&SAD, &SAD2, img, H, pattern_image, n);
              
              //check if the final image's results is within thresholds
              if (votes_ > votingThreshold_)
              {
                
                float MO = ( SAD + SAD2 ) / 2;
                if (MO < MOThreshold_)
                {
                  HazmatEpsilon a;
                  a.x = point.x;
                  a.y = point.y;
                  a.pattern_num = n+1;
                  a.m = m;
                  a.MO = MO;
                  a.votes = votes_;
                  a.H = cvCloneMat(H);
                  if (result.size() == 0)
                  {
                    result.push_back(a);
                  }
                  else
                  {
                    bool flag = false;
                    for (int v = 0; v < result.size() ; v++)
                    {
                      float check1 = fabsf( a.x - result[v].x);
                      float check2 = fabsf( a.y - result[v].y);
                      if ( (check1 <= 20) && (check2 <= 20) )
                      {
                        flag = true;
                        float possibility1 = 0;
                        float possibility2 = 0;
                        //check if i already have a hazmat in the same place
                        //and check which fits best
                        if ( a.m > result[v].m)
                        {
                          possibility1 = possibility1 + 0.4;
                        }
                        else
                        {
                          possibility2 = possibility2 +0.4;
                        }
                        
                        if (a.MO < result[v].MO)
                        {
                          possibility1 = possibility1 + 0.3;
                        }
                        else
                        {
                          possibility2 = possibility2 + 0.3;
                        }
                        
                        if ( a.votes > result[v].votes )
                        {
                          possibility1 = possibility1 + 0.3;
                        }
                        else if( a.votes == result[v].votes )
                        {
                          possibility1 = possibility1 + 0.3;
                          possibility2 = possibility2 + 0.3;
                        }
                        else
                        {
                          possibility2 = possibility2 + 0.3;
                        }
                        if ( possibility1 >= possibility2)
                        {
                          result[v].x = a.x;
                          result[v].y = a.y;
                          result[v].pattern_num = a.pattern_num;
                          result[v].m = a.m;
                          result[v].MO = a.MO;
                          result[v].votes = a.votes;
                          result[v].H = a.H;
                        }
                      }
                    }
                    if (flag == false){
                      result.push_back(a);
                    }
                  }
                }
              }
            }
          }
          cvReleaseMat(&H);
        }
        if (tempvector.size() > 0)
        {
          for (int l = 0 ; l < tempvector.size() ; l++)
          {
            feats_[n][tempvector[l]].fwd_match = NULL;
          }
        }
        tempvector.erase(tempvector.begin(), tempvector.end());
      }
      kdtree_release(kd_root);
      free(featShot);
    }

    delete [] feats_;
    delete temp;
    cvReleaseImage(&img);
    return result;
  }

  /**
  @brief the fully functional detector of both hazmat and epsilon patterns
  @param img [cv::Mat]
  @return std::vector<HazmatEpsilon>
  **/
  std::vector<HazmatEpsilon> HazmatEpsilonDetector::DetectHazmatEpsilon(
    cv::Mat img)
  {
    std::vector<HazmatEpsilon> result;
    result = detectHazmat(img);
    return result;
  }
} // namespace pandora_vision

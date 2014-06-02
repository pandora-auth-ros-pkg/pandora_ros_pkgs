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
   @brief Default Constructor 
  */ 
  HazmatEpsilonDetector::HazmatEpsilonDetector(std::string _package_path)
  {
    rows = DEFAULT_HEIGHT;
    cols = DEFAULT_WIDTH;
    
    package_path = _package_path;
    pattern_index_path = package_path + "/contents";
  
    initDetector();
    frameNum = 0;
  }
  
  /**
    @brief Initialize hazmat detector. Loads hazmats from hard disk \
    into memory.
    @return void
  **/
  void HazmatEpsilonDetector::initDetector()
  {
    FILE* contents = fopen(pattern_index_path.c_str(), "r");
    if (contents == NULL) 
      printf("Can't open contents file");
    
    nPatterns = 0;
        
    char imgName[50];
    while(true)
    {
      fgets(imgName, 49, contents);

      if(imgName[0] == '\n') 
        break;
      
      if (feof(contents) != 0) 
        break;

      nPatterns++;
    }
    /// Sets the position indicator associated with stream to the \
    beginning of the file.
    rewind(contents);
    
    /// Load features of patterns in memory
    feats = new feature* [nPatterns];
    nFeats = new int[nPatterns];
    
    for(int n = 0; n < nPatterns; n++)
    {
      fgets(imgName, 49, contents);
      imgName[strlen(imgName)-1] = '\0';
      
      std::stringstream img_file_stream;
      
      img_file_stream << package_path << "/" << imgName << ".sift";
    
      /// Returns the number of features imported from every pattern
      nFeats[n] = import_features(const_cast<char*>(img_file_stream.str().c_str())
        , FEATURE_LOWE, &feats[n]);
    } 
    fclose(contents);
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
    int clrVariance, float votingThr, float minAreaThr,
    float maxAreaThr, int sideLgth, int featThr, float MOThr)
  {
    colorVariance = clrVariance;
    votingThreshold = votingThr;
    minAreaThreshold = minAreaThr;
    maxAreaThreshold = maxAreaThr;
    featureThreshold = featThr;
    MOThreshold = MOThr;
    sideLength = sideLgth;

    calcMinMax();
  }
  
  /**
   @brief Default constructor
  */ 
  HazmatEpsilonDetector::~HazmatEpsilonDetector(){
    delete[] nFeats;
    delete[] minUV;
    delete[] maxUV;
  }
   
  /**
    @brief (?)
    @return void
  **/  
  void HazmatEpsilonDetector::calcMinMax()
  {
    minUV = new float* [nPatterns];
    maxUV = new float* [nPatterns];
    cv::Mat image;
    for (int i = 0; i < nPatterns; i++)
    {
      minUV[i] = new float [2];
      maxUV[i] = new float [2];
    }
    for (int i = 0; i < nPatterns; i++)
    {
      std::stringstream img_file_stream;
      img_file_stream << package_path << "/patterns/enter" << i+1 << ".png";
      
      image = cv::imread(img_file_stream.str().c_str(), 1);
      cvtColor(image, image, CV_BGR2YCrCb);
      
      int height, width, step, channels;
      height = image.size().height;
      width  = image.size().width;
      step = image.step;
      channels = image.channels();
      
      for (int j = 0; j < sideLength; j++)
      {
        for (int k = 0; k < sideLength; k++)
        {
          if( (j == 0) && (k == 0) ){
            minUV[i][0] = (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength / 2) + j) * step +
                ((width / 2) - (sideLength / 2) + k) * channels + 1];
            minUV[i][1] = (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength / 2) + j) * step +
                ((width / 2) - (sideLength / 2) + k) * channels + 2];
            maxUV[i][0] = (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength / 2) + j) * step +
                ((width / 2) - (sideLength / 2) + k) * channels + 1];
            maxUV[i][1] = (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength / 2) + j) * step +
                ((width / 2) - (sideLength / 2) + k) * channels + 2];
          }  
          else{
            if(minUV[i][0] > (reinterpret_cast<uchar *>(image.data))
              [((height / 2) - (sideLength / 2) + j) * step +
                ((width / 2) - (sideLength / 2) + k) * channels + 1])
            {
              minUV[i][0] = (reinterpret_cast<uchar *>(image.data))
                [((height / 2) - (sideLength / 2) + j) * step +
                  ((width / 2) - (sideLength / 2) + k) * channels + 1];
            }
            
            if (minUV[i][1] > (reinterpret_cast<uchar *>(image.data))[
              ((height / 2) - (sideLength / 2) + j) * step +
                ((width / 2) - (sideLength / 2) + k) * channels + 2])
            {
              minUV[i][1] = (reinterpret_cast<uchar *>(image.data))[
                ((height / 2) - (sideLength / 2) + j) * step +
                  ((width / 2) - (sideLength / 2) + k) * channels + 2];
            }
            
            if(maxUV[i][0] < (reinterpret_cast<uchar *>(image.data))[
              ((height / 2) - (sideLength / 2) + j) * step +
                ((width / 2) - (sideLength / 2) + k) * channels + 1])
            {
              maxUV[i][0] = (reinterpret_cast<uchar *>(image.data))[
                ((height / 2) - (sideLength / 2) + j) * step +
                  ((width / 2) - (sideLength / 2) + k) * channels + 1];
            }
            
            if(maxUV[i][1] < (reinterpret_cast<uchar *>(image.data))[
              ((height / 2) - (sideLength / 2) + j) * step +
                ((width / 2) - (sideLength / 2) + k) * channels + 2])
            {
              maxUV[i][1] = (reinterpret_cast<uchar *>(image.data))[
                ((height / 2) - (sideLength / 2) + j) * step +
                  ((width / 2) - (sideLength / 2) + k) * channels + 2];
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
  void HazmatEpsilonDetector::calcHistograms()
  {
    cv::Mat* patterns = new cv::Mat[nPatterns];
    cv::Mat image;
    
    for(int i = 0; i < nPatterns; i++)
    {
     std::stringstream img_file_stream;
     img_file_stream << package_path << "/patterns/enter" << i+1 << ".png";
     
     image = cv::imread(img_file_stream.str().c_str(), 1);
     cvtColor(image, image, CV_BGR2HSV);
     patterns[i] = image;
    }
    
    /// Quantize the hue to 30 levels and the saturation to 32 levels
    int hbins = 30;
    int histSize[] = {hbins};
    /// hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    const float* ranges[] = { hranges};
    /// Compute the histogram from the 0-th and 1-st channels
    int channels[] = {0};
    calcHist(patterns , nPatterns, channels, cv::Mat(), patternHistog, 1, 
      histSize, ranges, true, false );
    delete[] patterns;
  }

  /**
    @brief Calculates the area of a rectangle from its four corners
    @param pt1 [CvPoint2D64f]
    @param pt2 [CvPoint2D64f]
    @param pt3 [CvPoint2D64f]
    @param pt4 [CvPoint2D64f]
    @return float The area
  **/
  float HazmatEpsilonDetector::calculateRectangleArea(CvPoint2D64f pt1,
    CvPoint2D64f pt2, CvPoint2D64f pt3, CvPoint2D64f pt4)
  {
    float sideA, sideB, sideC; 
    sideA = sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
    sideB = sqrt(pow((pt2.x - pt3.x), 2) + pow((pt2.y - pt3.y), 2));
    sideC = sqrt(pow((pt1.x - pt3.x), 2) + pow((pt1.y - pt3.y), 2));
     
    float temp, triangleArea;
    
    temp = (sideA + sideB + sideC) / 2.0;
    
    triangleArea = sqrt( temp*(temp - sideA) * (temp - sideB) * (temp - sideC));
    
    float triangleArea2;
    sideA = sqrt(pow((pt1.x - pt3.x), 2) + pow((pt1.y - pt3.y), 2));
    sideB = sqrt(pow((pt1.x - pt4.x), 2) + pow((pt1.y - pt4.y), 2));
    sideC = sqrt(pow((pt3.x - pt4.x), 2) + pow((pt3.y - pt4.y), 2));
    
    temp = (sideA + sideB + sideC) / 2.0;
    
    triangleArea2 = sqrt(temp*(temp - sideA)*(temp - sideB)*(temp - sideC));
    
    float rectangleArea = triangleArea + triangleArea2;
    
    return rectangleArea;
  }

  /**
    @brief Reads contents from file "contents" and stores into memory \
    the processed hazmats
    @return void
  **/
  void HazmatEpsilonDetector::preprocessHazmat()
  {
    /// Read index of file contents
    FILE* contents = fopen(pattern_index_path.c_str(), "r");
    char imgName[50], featName[50];
    
    while( true )
    {
      // Get next pattern
      fgets(imgName, 49, contents);
      imgName[strlen(imgName)-1] = '\0';
      
      if( strlen(imgName) == 0) 
        break;
        
      ROS_INFO_STREAM("Processing hazmat" << imgName);
      
      struct feature* feat;

      /// Compute and store features
      IplImage* img = cvLoadImage(imgName, 1);
      if(!img) 
        ROS_ERROR("No input image!");
      
      
      int n = sift_features(img, &feat);
      snprintf(featName, sizeof(featName), "%s.sift", imgName);
      export_features(featName, feat, n);

      /// Free memory
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
  std::vector <int>HazmatEpsilonDetector::findFeature(int *m, int n, 
    int testNum, struct kd_node* kd_root)
  {
    struct feature** nbrs;
    
    std::vector <int> tempvector;
    
    for(int i = 0; i < testNum; i++)
    {
      int k = 
        kdtree_bbf_knn( kd_root, feats[n] +i, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
      if( k == 2 )
      {
        double d0 = descr_dist_sq( feats[n] +i, nbrs[0] );
        double d1 = descr_dist_sq( feats[n] +i, nbrs[1] );
        if( d0 < d1 * NN_SQ_DIST_RATIO_THR ){
          feats[n][i].fwd_match = nbrs[0];
          tempvector.push_back(i);
          m[0]++;
          
        }
      }
      free(nbrs);
    }
    return tempvector;
  } 
  
  /**
    @brief Calculates the area of a pattern in an image 
    @param H [CvMat*]
    @param pattern_image [cv::Mat]
    @return void
  **/
  void  HazmatEpsilonDetector::calculateArea(CvMat* H, cv::Mat pattern_image)
  {
    CvPoint2D64f point1 = {0, 0};
    CvPoint2D64f point2 = {0, pattern_image.size().height};
    CvPoint2D64f point3 = {pattern_image.size().width, 0};
    CvPoint2D64f point4 = {pattern_image.size().width, 
      pattern_image.size().height};
    
    /// Find the transformation of the 4 corners of the initial image
    point1 = persp_xform_pt( point1, H );
    point2 = persp_xform_pt( point2, H );
    point3 = persp_xform_pt( point3, H );
    point4 = persp_xform_pt( point4, H );
    /// Calculate the final area according to the given points
    area = calculateRectangleArea(point1, point2, point3, point4);
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
  CvPoint2D64f HazmatEpsilonDetector::defineVariance(float* SAD, 
    float* SAD2, IplImage* img, CvMat* H, cv::Mat _pattern_image, int n)  
  { 
    /// Convert cv::Mat to IplImage in order to use opensift library
    IplImage* pattern_image = 
      cvCreateImage( cvSize(cols, rows), IPL_DEPTH_8U, 3 );
    
    IplImage* temp = new IplImage(_pattern_image);
    pattern_image = cvCloneImage(temp);
     
    IplImage* _xformed = cvCreateImage( cvGetSize( img ), IPL_DEPTH_8U, 3 );
    cvWarpPerspective( pattern_image, _xformed, H, 
      CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cv::Scalar( 0, 0, 0));
    cv::Mat xformed(_xformed, false);
   
    cv::Mat color_image = cv::Mat( cv::Size(cols, rows), CV_8U, 3 );
    xformed.copyTo(color_image);
    cvtColor(color_image, color_image, CV_BGR2YCrCb);
    cv::Mat _img(img, false);
    
    cv::Mat ycrcb_image = cv::Mat( cvSize(cols, rows), CV_8U, 3 );
    _img.copyTo(ycrcb_image);
    cvtColor(ycrcb_image, ycrcb_image, CV_BGR2YCrCb);

    int color_image_height;
    int color_image_width;
    int color_image_step;
    int color_image_channels;
    
    color_image_height = color_image.size().height;
    color_image_width = color_image.size().width;
    color_image_step = color_image.step;
    color_image_channels = color_image.channels();
    
    int ycrycb_image_height;
    int ycrycb_image_width;
    int ycrycb_image_step;
    int ycrycb_image_channels;
    
    ycrycb_image_height = ycrcb_image.size().height;
    ycrycb_image_width = ycrcb_image.size().width;
    ycrycb_image_step = ycrcb_image.step;
    ycrycb_image_channels = ycrcb_image.channels();
    
    CvPoint2D64f point = 
      {(_pattern_image.size().width) / 2, (_pattern_image.size().height) / 2};
    point = persp_xform_pt( point, H );
    votes = 0;
              
    //start voting
    for (int counter = 0; counter < sideLength; counter++) {
      for (int counter2 = 0; counter2 < sideLength; counter2++){
        if (( (point.y - (sideLength / 2) + counter ) >= 0) && 
            ( (point.y - (sideLength / 2) + counter ) < rows ))
        {
          if (( (point.x - (sideLength / 2) + counter2 ) >= 0) && 
              ( (point.x - (sideLength / 2) + counter2 ) < cols ))
            {
              bool test1 = 
                ( (reinterpret_cast<uchar*>(ycrcb_image.data)
                [ ( static_cast<int>(point.y) - sideLength / 2 + counter) *
                  color_image_step +
                  ( static_cast<int>(point.x) - sideLength / 2 + counter2) *
                    color_image_channels + 1 ] >= (minUV[n][0] - 333333333333333333333))
              &&
              ( (reinterpret_cast<uchar*>(ycrcb_image.data)
                [ ( static_cast<int>(point.y) - sideLength / 2 + counter) *
                    color_image_step +
                  ( static_cast<int>(point.x) - sideLength / 2 + counter2) *
                    color_image_channels + 1] < (maxUV[n][0] + colorVariance))));
                  
              if(test1 == true)
              {
                bool test2 =
                ( (reinterpret_cast<uchar*>(ycrcb_image.data)
                [ ( static_cast<int>(point.y) - sideLength / 2 + counter) *
                  color_image_step +
                  ( static_cast<int>(point.x) - sideLength / 2 + counter2) *
                    color_image_channels + 2 ]>= (minUV[n][1] - colorVariance)))
                &&
                ( (reinterpret_cast<uchar*>(ycrcb_image.data))
                  [ ( static_cast<int>(point.y) - sideLength / 2 + counter) *
                      color_image_step +
                    ( static_cast<int>(point.x) - sideLength / 2 + counter2) *
                      color_image_channels + 2] <= (maxUV[n][1] + colorVariance));
                if(test2 == true)
                  votes = votes + 1;
              }
              
            /// Find the absolute variance between the initial and the final image
            *SAD = *SAD + fabsf(
              (reinterpret_cast<uchar*>(ycrcb_image.data))[
                ( static_cast<int>(point.y) - sideLength / 2 + counter) *
                  ycrycb_image_step +
                ( static_cast<int>(point.x) - sideLength / 2 + counter2) *
                  ycrycb_image_channels + 1] -
              (reinterpret_cast<uchar*>(color_image.data))[
                ( static_cast<int>(point.y) - sideLength / 2 + counter) *
                  color_image_step +
                ( static_cast<int>(point.x) - sideLength / 2 + counter2) *
                  color_image_channels + 1] );
                  
            *SAD2 = *SAD2 + fabsf(
            (reinterpret_cast<uchar*>(ycrcb_image.data))[
                ( static_cast<int>(point.y) - sideLength / 2 + counter) *
                  ycrycb_image_step +
                ( static_cast<int>(point.x) - sideLength / 2 + counter2) *
                  ycrycb_image_channels + 2] -
              (reinterpret_cast<uchar*>(color_image.data))[
                ( static_cast<int>(point.y) - sideLength / 2 + counter) *
                  color_image_step +
                ( static_cast<int>(point.x) - sideLength / 2 + counter2) *
                  color_image_channels + 2] );
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
    @brief The core of hazmat detector
    @param img [cv::Mat] Current frame to be processed
    @return std::vector<HazmatEpsilon> of found hazmats in current frame
  */ 
  std::vector<HazmatEpsilon> HazmatEpsilonDetector::detectHazmat
    (cv::Mat hazmatFrame)
  {
    std::vector<HazmatEpsilon> result;
    
    IplImage* img = cvCreateImage( cv::Size(cols, rows), IPL_DEPTH_8U, 3 );
    IplImage* temp = new IplImage(hazmatFrame);
    img = cvCloneImage(temp);
      
    initDetector();

    /// SIFT process of screenshot
    struct kd_node* kd_root;
    
    if (!img)
      ROS_ERROR("Could not load image");
      
    struct feature* featShot;
    
    /// Finds SIFT features in an image.
    int nShot = sift_features(img, &featShot);
    
    if (nShot > 0)
    {
      kd_root = kdtree_build( featShot, nShot );
      
      /// Search screenshot for each pattern
      int i;
      int max = -1;
      for( int n = 0; n < nPatterns; n++)
      {
        /// Find nearest neighboor for each feature in screenshot
        int m = 0;
        /// Number of imported features from each image
        int num_of_features = nFeats[n];
        
        std::vector <int> feature_vector;
        feature_vector = findFeature(&m, n, num_of_features, kd_root);
   
        /// Run RANSAC to find out a transformation that transforms
        /// patterns into screenshot
        if( m > featureThreshold )
        {
          CvMat* H;
          H = ransac_xform( feats[n], num_of_features, FEATURE_FWD_MATCH, 
            lsq_homog, 4, 0.01, homog_xfer_err, 3.0, NULL, NULL );
          if( H )
          {
            std::stringstream img_file_stream;
            img_file_stream << package_path << "/patterns/enter" 
              << n+1  << ".png";
            
            cv::Mat pattern_image;
            pattern_image = 
              cv::imread(img_file_stream.str().c_str(), CV_LOAD_IMAGE_COLOR);
            if (!pattern_image.data)
              ROS_ERROR("could not load pattern image");

            calculateArea(H, pattern_image);
            
            if (area >= minAreaThreshold && area <= maxAreaThreshold)
            {
              float SAD = 0;
              float SAD2 = 0;
              CvPoint2D64f point;
              point = defineVariance(&SAD, &SAD2, img, H, pattern_image, n);
              
              ///check if the final image's results is within thresholds
              if (votes > votingThreshold){
                
                float MO = ( SAD + SAD2 )/2;
                if (MO < MOThreshold)
                {
                  HazmatEpsilon a;
                  a.x = point.x;
                  a.y = point.y;
                  a.pattern_num = n+1;
                  a.m = m;
                  a.MO = MO;
                  a.votes = votes;
                  a.H = cvCloneMat(H);
                  if (result.size() == 0)
                    result.push_back(a);
                  else
                  {
                    bool flag = false;
                    for (int v = 0; v < result.size(); v++)
                    {
                      float check1 = fabsf( a.x - result[v].x);
                      float check2 = fabsf( a.y - result[v].y);
                      if ( (check1 <= 20) && (check2 <= 20) )
                      {
                        flag = true;
                        float isHazmat = 0;
                        float isNotHazmat = 0;
                        ///Check if i already have a hazmat in the same place 
                        ///and check which fits best
                        if ( a.m > result[v].m)
                          isHazmat = isHazmat + 0.4;
                        else if(a.m  ==  result[v].m){
                          isHazmat = isHazmat + 0.4;
                          isNotHazmat = isNotHazmat + 0.4;
                        } 
                        else
                          isNotHazmat = isNotHazmat +0.4;
                          
                        if (a.MO < result[v].MO)
                          isHazmat = isHazmat + 0.3;
                        else if( a.votes == result[v].votes )
                        {
                          isHazmat = isHazmat + 0.3;
                          isNotHazmat = isNotHazmat + 0.3;
                        }  
                        else
                          isNotHazmat = isNotHazmat + 0.3;
                          
                        if ( a.votes > result[v].votes ){
                          isHazmat = isHazmat + 0.3;
                        }
                        else if( a.votes == result[v].votes )
                        {
                          isHazmat = isHazmat + 0.3;
                          isNotHazmat = isNotHazmat + 0.3;
                        }
                        else
                          isNotHazmat = isNotHazmat + 0.3;
                        
                        if ( isHazmat >= isNotHazmat)
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
        if (feature_vector.size() > 0)
        {
          for (int l = 0; l < feature_vector.size(); l++)
          {
            feats[n][feature_vector[l]].fwd_match = NULL;
          }
        }
        feature_vector.erase(feature_vector.begin(), feature_vector.end());
      }
      kdtree_release(kd_root);
      free(featShot);
    }
    delete[] feats;
    delete temp;
    cvReleaseImage(&img);
    return result;
  }
  
}// namespace pandora_vision

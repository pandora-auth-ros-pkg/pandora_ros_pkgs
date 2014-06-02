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
* Author:  Tsakalis Vasilis
*          Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_hazmat/sift.h"
#include "pandora_vision_hazmat/imgfeatures.h"
#include "pandora_vision_hazmat/utils.h"

#include <highgui.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include "ros/ros.h"
#include <ros/package.h>

static void usage( char* name );
static void modify_arguements();
static int export_lowe_features( const char*, struct feature*, int );

char* pname;
const char* out_img_name = NULL;
int intvls = SIFT_INTVLS;
double sigma = SIFT_SIGMA;
double contr_thr = SIFT_CONTR_THR;
int curv_thr = SIFT_CURV_THR;
int img_dbl = SIFT_IMG_DBL;
int descr_width = SIFT_DESCR_WIDTH;
int descr_hist_bins = SIFT_DESCR_HIST_BINS;

int main( int argc, char** argv )
{   
    
    IplImage* img;
    struct feature* features;
    int n = 0;
    modify_arguements();
       
    std::string packagePath = ros::package::getPath("pandora_vision_hazmat");
    int numOfPatterns = 0;
    std::cout << "Give the number of patterns you want to train" << std::endl;
    std::cin >> numOfPatterns;

    for(int i = 0; i < numOfPatterns; i++)
    {
      std::stringstream img_file_stream;
      
      img_file_stream << packagePath << "/patterns/enter" << i+1 << ".png";
      std::string img_file_name = img_file_stream.str();
      
      img = cvLoadImage( img_file_name.c_str(), 1 );
      if(!img)
        std::cout << "Unable to load image from %s" << img_file_name;
        
      std::cout <<"Finding SIFT features...\n";

      n = _sift_features( img, &features, intvls, sigma, contr_thr, curv_thr,
        img_dbl, descr_width, descr_hist_bins );
      std::cout << "Found " << n <<"features." << std::endl;
       
      draw_features( img, features, n );
      
      cv::Mat image(img);
      cv::imshow("Hazmat", image);
      cv::waitKey(50);
      
      std::stringstream out_file_stream;
      
      out_file_stream << packagePath << "/patterns/enter" << i+1 << ".png.sift";
      out_img_name = (out_file_stream.str()).c_str();
      
      int result;
      result = export_lowe_features(out_img_name, features, n );
      if(result == 0)
        printf("Feautures of %d pattern are added successfully,\n", i);
      
      std::stringstream sift_image_file_stream;
      
      sift_image_file_stream << packagePath << "/patterns/sift/enter" << i+1 << ".png";
            
      cv::imwrite(sift_image_file_stream.str(), image);
    }  
    return 0;
}

static int export_lowe_features( const char* filename, struct feature* feat, int n )
{
    FILE* file;
    int i, j, d;

    if( n <= 0 )
    {
      fprintf( stderr, "Warning: feature count %d, %s, line %d\n",
         n, __FILE__, __LINE__ );
      return 1;
    }
    file = fopen( filename, "w");
    
    if(file == NULL)
    {
      fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
         filename, __FILE__, __LINE__ );
      return 1;
    }

    d = feat[0].d;
    fprintf( file, "%d %d\n", n, d );
    for( i = 0; i < n; i++ )
    {
      fprintf( file, "%f %f %f %f", feat[i].y, feat[i].x,
         feat[i].scl, feat[i].ori );
      for( j = 0; j < d; j++ )
    {
      /* write 20 descriptor values per line */
      if( j % 20 == 0 )
      fprintf( file, "\n" );
      fprintf( file, " %d", static_cast<int>(feat[i].descr[j]) );
    }
      fprintf( file, "\n" );
    }

    if( fclose(file) )
    {
      fprintf( stderr, "Warning: file close error, %s, line %d\n",
         __FILE__, __LINE__ );
      return 1;
    }
    return 0;
}


/// Print usage for this program
static void usage( char* name )
{
  fprintf(stderr, "Usage: %s detects SIFT keypoints in an image", name);
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "  -h               Display this message and exit\n");
  fprintf(stderr, "  -i <intervals>   Set number of sampled intervals per" \
    " octave in scale space\n");
  fprintf(stderr, "                   pyramid (default %d)\n",
    SIFT_INTVLS);
  fprintf(stderr, "  -s <sigma>       Set sigma for initial gaussian" \
    " smoothing at each octave\n");
  fprintf(stderr, "                   (default %06.4f)\n", SIFT_SIGMA);
  fprintf(stderr, "  -c <thresh>      Set threshold on keypoint contrast" \
    " |D(x)| based on [0,1]\n");
  fprintf(stderr, "                   pixel values (default %06.4f)\n",
    SIFT_CONTR_THR);
  fprintf(stderr, "  -r <thresh>      Set threshold on keypoint ratio of" \
    " principle curvatures\n");
  fprintf(stderr, "                   (default %d)\n", SIFT_CURV_THR);
  fprintf(stderr, "  -n <width>       Set width of descriptor histogram" \
    " array (default %d)\n", SIFT_DESCR_WIDTH);
  fprintf(stderr, "  -b <bins>        Set number of bins per histogram" \
    " in descriptor array\n");
  fprintf(stderr, "                   (default %d)\n", SIFT_DESCR_HIST_BINS);
  fprintf(stderr, "  -x               Turn off keypoint display\n");
}


static void modify_arguements()
{
  usage("hazmat_trainer");
  char arg;
  std::cout << 
    "Select suitable option if you want to modify parameters or x to exit" 
      << std::endl;
  bool isModified = false;
  while( isModified == false)
  {
      std::cin >> arg;
      switch( arg )
      {

        /// read intervals
        case 'i':
          std::cout <<
            "Modify number of sampled intervals per octave" << std::endl;
          std::cin >> intvls;
          break;

        /// read sigma
        case 's' :
          std::cout << 
            "Modify sigma for initial gaussian smoothing at each octave" << std::endl;
          std::cin >> sigma;
          break;

        /// read contrast_thresh
        case 'c' :
          std::cout << 
            "Modify threshold on keypoint contrast|D(x)| based on [0,1]" << std::endl;
          /// parse argument and ensure it is a floating point number
          std::cin >> contr_thr;
          break;

        /// read curvature_thresh
        case 'r' :
          std::cout << 
            "Modify threshold on keypoint ratio of principle curvatures" << std::endl;
          /// parse argument and ensure it is a floating point number
          std::cin >> curv_thr;
          break;

        /// read descr_width
        case 'n' :
          std::cout << 
            "Modify width of descriptor histogram array" << std::endl;
          /// parse argument and ensure it is a floating point number
          std::cin >> descr_width;
          break;

        /// read descr_histo_bins
        case 'b' :
          std::cout << 
            "Modify number of bins per histogram in descriptor array" << std::endl;
          std::cin >> descr_hist_bins;
          break;

        /// read display
        case 'x' :
          isModified = true;
          break;

        /// user asked for help
        case 'h':
          usage( "hazmat_trainer");
          break;

        /// catch invalid arguments
        default:
          fatal_error( "-%c: invalid option.\nTry '%s -h' for help.",
          optopt, pname );
      }
      std::cout << 
        "Select next variable to modify or press x to exit" << std::endl;
    }
}

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
* 		   Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_hazmat/sift.h"
#include "pandora_vision_hazmat/imgfeatures.h"
#include "pandora_vision_hazmat/utils.h"

#include <highgui.h>
#include <iostream>
#include <unistd.h>
#include  <string.h>
#include "ros/ros.h"
#include <ros/package.h>
#define OPTIONS ":o:m:i:s:c:r:n:b:dxh"

using namespace std;
using namespace cv;
/*************************** Function Prototypes *****************************/

static void usage( char* name );
static void arg_parse( int argc, char** argv);

static int export_lowe_features( char*, struct feature*, int );
/******************************** Globals ************************************/

char* pname;
char* img_file_name;
char* out_file_name = NULL;
char* out_img_name = NULL;
int intvls = SIFT_INTVLS;
double sigma = SIFT_SIGMA;
double contr_thr = SIFT_CONTR_THR;
int curv_thr = SIFT_CURV_THR;
int img_dbl = SIFT_IMG_DBL;
int descr_width = SIFT_DESCR_WIDTH;
int descr_hist_bins = SIFT_DESCR_HIST_BINS;
int display = 1;


/********************************** Main *************************************/

int main( int argc, char** argv )
{
	  IplImage* img;
	  struct feature* features;
	  int n = 0;
	  
	  std::string packagePath = ros::package::getPath("pandora_vision_hazmat");
	  int numOfPatterns=0;
	  printf("Give the number of patterns you want to train");
	  std::cin>>numOfPatterns;
	  
	  std::string filename1;
	  std::string filename2;
	  
	  for(int i=0;i<numOfPatterns;i++)
	  {
		char temp_name1[200];
		char temp_name2[200];
		filename1=packagePath;
		filename2=packagePath;
		
		sprintf(temp_name1,"/patterns/enter%d.png",i+1);
		std::string temp1(temp_name1);
		filename1.append(temp1);
		
		img = cvLoadImage( filename1.c_str(), 1 );
		printf( "Finding SIFT features...\n" );
		
		
		sprintf(temp_name2,"/patterns/enter%d.png.sift",i+1);
		std::string temp2(temp_name2);
		filename2.append(temp2);
		
		n = sift_features( img, &features);
		printf( "Found %d features.\n", n );
	  
		
		int result;
		char* cstr =new char[200];
		strcpy(cstr, filename2.c_str());
		result= export_lowe_features(cstr, features, n );
		if(result==0)
		{
			printf("Feautures of %d pattern are added successfully,\n",i);
		}
		  filename1.clear();
		  filename2.clear();
	}
	  
	  return 0;
}

static int export_lowe_features( char* filename, struct feature* feat, int n )
{
	  FILE* file;
	  int i, j, d;

	  if( n <= 0 )
		{
		  fprintf( stderr, "Warning: feature count %d, %s, line %d\n",
			   n, __FILE__, __LINE__ );
		  return 1;
		}
	  if( ! ( file = fopen( filename, "w" ) ) )
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
		  fprintf( file, " %d", (int)(feat[i].descr[j]) );
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


/************************** Function Definitions *****************************/

// print usage for this program
static void usage( char* name )
	{
	  fprintf(stderr, "%s: detect SIFT keypoints in an image\n\n", name);
	  fprintf(stderr, "Usage: %s [options] <img_file>\n", name);
	  fprintf(stderr, "Options:\n");
	  fprintf(stderr, "  -h               Display this message and exit\n");
	  fprintf(stderr, "  -o <out_file>    Output keypoints to text file\n");
	  fprintf(stderr, "  -m <out_img>     Output keypoint image file (format" \
		  " determined by extension)\n");
	  fprintf(stderr, "  -i <intervals>   Set number of sampled intervals per" \
		  " octave in scale space\n");
	  fprintf(stderr, "                   pyramid (default %d)\n",
		  SIFT_INTVLS);
	  fprintf(stderr, "  -s <sigma>       Set sigma for initial gaussian"	\
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
	  fprintf(stderr, "  -d               Toggle image doubling (default %s)\n",
		  SIFT_IMG_DBL == 0 ? "off" : "on");
	  fprintf(stderr, "  -x               Turn off keypoint display\n");
}



/*
  arg_parse() parses the command line arguments, setting appropriate globals.
  
  argc and argv should be passed directly from the command line
*/
static void arg_parse( int argc, char** argv )
{
  //extract program name from command line (remove path, if present)
  pname = basename( argv[0] );

  //parse commandline options
  while( 1 )
    {
      char* arg_check;
      int arg = getopt( argc, argv, OPTIONS );
      if( arg == -1 )
	break;

      switch( arg )
	{
	  // catch unsupplied required arguments and exit
	case ':':
	  fatal_error( "-%c option requires an argument\n"		\
		       "Try '%s -h' for help.", optopt, pname );
	  break;

	  // read out_file_name
	case 'o':
	  if( ! optarg )
	    fatal_error( "error parsing arguments at -%c\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  out_file_name = optarg;
	  break;

	  // read out_img_name
	case 'm':
	  if( ! optarg )
	    fatal_error( "error parsing arguments at -%c\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  out_img_name = optarg;
	  break;
	  
	  // read intervals
	case 'i':
	  // ensure argument provided
	  if( ! optarg )
	    fatal_error( "error parsing arguments at -%c\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  
	  // parse argument and ensure it is an integer
	  intvls = strtol( optarg, &arg_check, 10 );
	  if( arg_check == optarg  ||  *arg_check != '\0' )
	    fatal_error( "-%c option requires an integer argument\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  break;
	  
	  // read sigma
	case 's' :
	  // ensure argument provided
	  if( ! optarg )
	    fatal_error( "error parsing arguments at -%c\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  
	  // parse argument and ensure it is a floating point number
	  sigma = strtod( optarg, &arg_check );
	  if( arg_check == optarg  ||  *arg_check != '\0' )
	    fatal_error( "-%c option requires a floating point argument\n" \
			 "Try '%s -h' for help.", arg, pname );
	  break;
	  
	  // read contrast_thresh
	case 'c' :
	  // ensure argument provided
	  if( ! optarg )
	    fatal_error( "error parsing arguments at -%c\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  
	  // parse argument and ensure it is a floating point number
	  contr_thr = strtod( optarg, &arg_check );
	  if( arg_check == optarg  ||  *arg_check != '\0' )
	    fatal_error( "-%c option requires a floating point argument\n" \
			 "Try '%s -h' for help.", arg, pname );
	  break;
	  
	  // read curvature_thresh
	case 'r' :
	  // ensure argument provided
	  if( ! optarg )
	    fatal_error( "error parsing arguments at -%c\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  
	  // parse argument and ensure it is a floating point number
	  curv_thr = strtol( optarg, &arg_check, 10 );
	  if( arg_check == optarg  ||  *arg_check != '\0' )
	    fatal_error( "-%c option requires an integer argument\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  break;
	  
	  // read descr_width
	case 'n' :
	  // ensure argument provided
	  if( ! optarg )
	    fatal_error( "error parsing arguments at -%c\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  
	  // parse argument and ensure it is a floating point number
	  descr_width = strtol( optarg, &arg_check, 10 );
	  if( arg_check == optarg  ||  *arg_check != '\0' )
	    fatal_error( "-%c option requires an integer argument\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  break;
	  
	  // read descr_histo_bins
	case 'b' :
	  // ensure argument provided
	  if( ! optarg )
	    fatal_error( "error parsing arguments at -%c\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  
	  // parse argument and ensure it is a floating point number
	  descr_hist_bins = strtol( optarg, &arg_check, 10 );
	  if( arg_check == optarg  ||  *arg_check != '\0' )
	    fatal_error( "-%c option requires an integer argument\n"	\
			 "Try '%s -h' for help.", arg, pname );
	  break;
	  
	  // read double_image
	case 'd' :
	  img_dbl = ( img_dbl == 1 )? 0 : 1;
	  break;

	  // read display
	case 'x' :
	  display = 0;
	  break;

	  // user asked for help
	case 'h':
	  usage( pname );
	  exit(0);
	  break;

	  // catch invalid arguments
	default:
	  fatal_error( "-%c: invalid option.\nTry '%s -h' for help.",
		       optopt, pname );
	}
    }

  //~ // make sure an input file is specified
  //~ if( argc - optind < 1 )
    //~ fatal_error( "no input file specified.\nTry '%s -h' for help.", pname );
//~ 
  //~ // make sure there aren't too many arguments
  //~ if( argc - optind > 1 )
    //~ fatal_error( "too many arguments.\nTry '%s -h' for help.", pname );

  // copy image file name from command line argument
  //img_file_name = argv[optind];
  img_file_name = "/home/despoina/enter8.png";
}

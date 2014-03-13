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
* Author: Michael Skolarikis
*********************************************************************/
/*
 * File Description :
 *	Generic class used to describe an area and
 *  calculate certain characteristics. Actually 
 *  is a class describing Blobs
 * 
 */ 
#include "pandora_vision_hole/thing.h"

namespace pandora_vision
{
  /**
   * Constructor
   */
  Thing::Thing()
  {		
    m_id = (uintptr_t)this;
    m_imgBlob = NULL;
    m_active = false;
    frameId = -1;
    probability = 0;
    
    //cout << "Thing's constructor called" << endl;
      
    //set chainHist values to 0
    for (int i=0; i<8; i++)
    {
      for (int j=0; j<8; j++)
      {
        m_chainHist[i][j] = 0;
      }
    }
  }

  /**
   * Destructor
   */
  Thing::~Thing()
  {	
    //cout << "Thing's destructor called" << endl;
  }

  /**
   * Copies itself to another thing.
   * @param destination
   */
  void Thing::cloneToThing(Thing* destination)
  {
    destination = new Thing;
    destination->createThing(m_imgBlob , m_contour);
  }

  /**
   * Creates a new "thing"
   * @param img
   * @param contour
   */
  void Thing::createThing(cv::Mat img , std::vector<cv::Point> contour)
  {
    
    m_contour = contour;
    m_imgBlob = cvCreateImage( img.size(), 8 , 1);
    IplImage local = img;
    cvCopy(&local,m_imgBlob);
    
    moments = cv::moments(img, true);
    m00 = moments.m00;
    m10 = moments.m10;
    m01 = moments.m01;
    m11 = moments.m11;
    m02 = moments.m02;
    m20 = moments.m20;
    
    mu20 = moments.mu20;
    mu11 = moments.mu11;
    mu02 = moments.mu02;
    mu30 = moments.mu30;
    mu21 = moments.mu21;
    mu12 = moments.mu12;
    mu03 = moments.mu03;
    
    //Pixel Area of Blob
    area = m00;
    
    //Pixel Perimeter of Blob
    perimeter = arcLength(contour, true);
    
    //Center of Gravity
    center = cvPoint2D32f(m10 / m00, m01 / m00);
    
    //Major-Axis
    maj_axis = 2 * sqrt( (mu20 + mu02 + sqrt( pow((mu20-mu02),2) + 4*pow(mu11,2)) ) / (m00/2) );
    
    //Minor-Axis
    min_axis = 2 * sqrt( (mu20 + mu02 - sqrt( pow((mu20-mu02),2) + 4*pow(mu11,2)) ) / (m00/2) );
    
    //Eccentricity of the ellipses	
    eccentricity = pow( ( (maj_axis - min_axis) / (maj_axis+min_axis) ) , 2 );
    
    ellipseArea = CV_PI * min_axis * maj_axis / 4;
    
    formFactor = ( 4 * CV_PI * area ) / ( pow(perimeter,2) );

    m_rect = cv::boundingRect(contour);
  }

  //void Thing::calculateHuMoments()
  //{
  //	cvGetHuMoments(&moments, &hu_moments);
  //}

  /**
   * Set thing's pattern type
   * @param type
   */
  void Thing::setPattern(int type)
  {
    this->m_pattern.isPattern = true;
    if (type == 0)
      this->m_pattern.isPositive = true;
    else if (type == 1)
      this->m_pattern.isPositive = false;
    else
      ROS_ERROR("Wrong pattern type");
  }

  //void Thing::calculateChainCode()
  //{
  //	IplImage* imgTemp = (IplImage*)cvClone(this->m_imgBlob);
  //
  //	m_freemanChain = 0;
  //	cvFindContours( imgTemp, m_storage_cch, (CvSeq**)&m_freemanChain, sizeof(*m_freemanChain), CV_RETR_LIST, CV_CHAIN_CODE);
  //
  //	int chainsize = m_freemanChain->elem_size;
  //	//cout << "chainsize is " << chainsize << endl;
  //
  //	CvSeq* current;
  //	for ( current=(CvSeq*)m_freemanChain; current != 0; current = current->h_next)
  //	{
  //		CvChainPtReader reader;
  //		cvStartReadChainPoints( (CvChain*)current, &reader );
  //
  //		/*
  //		IplImage* test1 = cvCreateImage(cvSize(640,480) ,8 ,3);
  //        cvZero( test1 );
  //		cvDrawContours( test1, current, cvScalarAll(255), cvScalarAll(0), -1, CV_FILLED, 8, cvPoint(0,0));
  //		cvShowImage("!!!" , test1);
  //		cvWaitKey(0);
  //		cvReleaseImage(&test1);
  //        */
  //
  //		int total = current->total;
  //		//cout << "total points are " << total << endl;
  //		for (int i = 0; i < total; i++)
  //		{
  //			CV_READ_SEQ_ELEM(reader.code, (*((CvSeqReader*)&(reader))));
  //			m_chainHist[0][reader.code]++;
  //			//printf("%d \n", reader.code);
  //		}
  //
  //		break;
  //	}
  //
  //	int sum = 0;
  //	for (int i=0; i<8; i++)
  //		sum = sum + m_chainHist[0][i];
  //
  //	for (int i=0; i<8; i++)
  //		cout << m_chainHist[0][i] << " ";
  //
  //	cout << endl;
  //
  //	for (int i=0; i<8; i++)
  //		cout << m_chainHist[0][i]/sum << " ";
  //
  //	cvReleaseImage(&imgTemp);
  //}

  /// Destroy Thing and deallocate memory 
  void Thing::destroyThing()
  {
    cvReleaseImage(&m_imgBlob);
    
    //cout << "Thing destroyed" << endl;
  }

  /**
   * Returns isPattern boolean
   * @return
   */
  bool Thing::isPattern()
  {
    return this->m_pattern.isPattern;
  }

  /**
   * Returns isPositive boolean
   * @return
   */
  bool Thing::isPatternPositive()
  {
    return this->m_pattern.isPositive;
  }

  /**
   * Prints info about the thing to the console.
   */
  void Thing::printInfo()
  {
  //	cout << "m00 " << m00 << endl;
  //	cout << "m01 " << m01 << endl;
  //	cout << "m10 " << m10 << endl;
  //	cout << "m11 " << m11 << endl;
  //	cout << "m20 " << m20 << endl;
  //	cout << "m02 " << m02 << endl;
    
  //	cout << "mu20 " << mu20 << endl;
  //	cout << "mu11 " << mu11 << endl;
  //	cout << "mu02 " << mu02 << endl;
  //	cout << "mu03 " << mu03 << endl;
  //	cout << "mu21 " << mu21 << endl;
  //	cout << "mu12 " << mu12 << endl;
  //	cout << "mu30 " << mu30 << endl;

    ROS_INFO("Bounding rect width,height %d,%d",m_rect.x, m_rect.y );
    ROS_INFO("Perimeter %f", perimeter);
    ROS_INFO("Area %f ", m00); 
    ROS_INFO("Form Factor %f ", formFactor);
    ROS_INFO("Eccentricity %f", eccentricity);
    ROS_INFO("Major %f ",maj_axis);
    ROS_INFO("Minor %f ", min_axis);
    ROS_INFO("Center (%f,%f)", center.x,center.y);
  }
}


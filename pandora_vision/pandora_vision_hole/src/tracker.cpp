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
* Author: George Aprilis, Michael Skolarikis
*********************************************************************/

#include "pandora_vision_hole/tracker.h"

namespace pandora_vision
{

/**
 * Constructor
 */ 
Tracker::Tracker()
{		
	std::cout << "[Tracker] : ---Tracker Parameters---" << std::endl;
	std::cout << "[Tracker] : L1 " << L1 << std::endl;
	std::cout << "[Tracker] : L1 " << L2 << std::endl;
	std::cout << "[Tracker] : L1 " << L3 << std::endl;
	std::cout << "[Tracker] : W1 " << W1 << std::endl;
	std::cout << "[Tracker] : W2 " << W2 << std::endl;
	std::cout << "[Tracker] : W3 " << W3 << std::endl;
	std::cout << "[Tracker] : THRESHOLD " << THRESHOLD << std::endl;
	std::cout << "[Tracker] : MIN_INACTIVITY " << MIN_INACTIVITY << std::endl;
	std::cout << "[Tracker] : MAX_INACTIVITY " << MAX_INACTIVITY << std::endl;
	std::cout << "[Tracker] : MAX_CHAIN_BUFFER " << MAX_CHAIN_BUFFER << std::endl;
	std::cout << "[Tracker] : PROBABILITY_BLOB_NUM " << PROBABILITY_BLOB_NUM << std::endl;
	std::cout << "[Tracker] : PROBABILITY_MAX_BLOB_AGE " << PROBABILITY_MAX_BLOB_AGE << std::endl;
	std::cout << "[Tracker] : CENTER_REGRESSION_ERROR " << CENTER_REGRESSION_ERROR << std::endl;
	std::cout << "[Tracker] : AREA_REGRESSION_ERROR " << AREA_REGRESSION_ERROR << std::endl;
	std::cout << "[Tracker] : OVERLAP_REGRESSION_ERROR " << OVERLAP_REGRESSION_ERROR << std::endl;
	std::cout << "[Tracker] : MAX_AGE_BLOB_ESTIMATE " << MAX_AGE_BLOB_ESTIMATE << std::endl;
	std::cout << "[Tracker] : OVERLAP_DILATION_ITERATIONS " << OVERLAP_DILATION_ITERATIONS << std::endl;
	std::cout << "[Tracker] : SEND_ALL " << SEND_ALL << std::endl;
	std::cout << "[Tracker] : ---End of Tracker Parameters---" << std::endl;

	/// clear chainList
	chainList.clear();

	/// clear camshift blob list
	camShiftBlobs.clear();

	/// set camshift parameters
	csVmin = 0;
	csVmax = 0;
	csSmax = 0;
	csSmin = 0;

	///allocate images for camshift
	csBackproject = cvCreateImage( cvSize(640,480), 8, 1 );
	csImage = cvCreateImage( cvSize(640,480), 8, 3 );
	csHsv = cvCreateImage( cvSize(640,480), 8, 3 );
	csHue = cvCreateImage( cvSize(640,480), 8, 1 );
	csMask = cvCreateImage( cvSize(640,480), 8, 1 );
	csAnd = cvCreateImage( cvSize(640,480), 8, 1 );
	/*
	cvStartWindowThread();
	cvNamedWindow("Camshift", CV_WINDOW_NORMAL);
	cvCreateTrackbar("csVmin", "Camshift", &this->csVmin, 255, NULL);
	cvCreateTrackbar("csVmax", "Camshift", &this->csVmax, 255, NULL);
	cvCreateTrackbar("csSmin", "Camshift", &this->csSmin, 255, NULL);
	cvCreateTrackbar("csSmax", "Camshift", &this->csSmax, 255, NULL);
	 */
	/// create blobList and currentList vectors
	blobList = new std::vector<Thing*>;
	currentList = new std::vector<Thing*>;

	/// create the buffer to hold 5 images, showing
	/// the last blobs of 5 chains
	imageBuffer = new IplImage*[5];
	for (int i=0; i<5; i++)
		imageBuffer[i] = cvCreateImage( cvSize(640,480) , 8 , 3);

	ROS_INFO("[Tracker] : New Tracker created!");
}

/**
 *  Destructor
 */ 
Tracker::~Tracker()
{
	ROS_INFO("[Tracker] : Tracker destroyed!"); 

	for (int i=0; i<5; i++)
		cvReleaseImage(&imageBuffer[i]);
	delete imageBuffer;

	///deallocate images of camShift tracker
	cvReleaseImage(&csImage);
	cvReleaseImage(&csHsv);
	cvReleaseImage(&csHue);
	cvReleaseImage(&csMask);
	cvReleaseImage(&csBackproject);
	cvReleaseImage(&csAnd);

	delete blobList;
	delete currentList;
}

/**
 *  Tracking Operation for a hole. A hole gets tracked so we can increase its possibility and
 *  inform everyone where it is.
 * @param list
 * @param dst
 * @param count
 */ 
void Tracker::track( std::vector<Thing*>* list , std::vector<Thing*>* dst , int count)
{	

	//cout << "[Tracker] : ---New tracking operation initiated---" << endl << endl;

	/// set frameNum info for every blob
	/// count is an increasing number 
	frameNum = count;

	/// clear list of estimated blobs		
	this->blobList->clear();	

	/// clear currentList
	this->currentList->clear();

	/// if input list is not empty (NULL)
	if (list) 	
	{		
		/// fill currentList
		/// for every Blob in the input list, set info,
		/// about the frame they were observed
		for (int i=0; i<list->size(); i++)
		{
			Thing* current = list->at(i);
			current->frameId = count;
			this->currentList->push_back( current );
		}

		/// if the list of chains is not empty
		/// calculate an estimation of the position
		/// for the next blob of this chain. Estimate
		/// where the next blob should be observed 
		/// at the current frame.
		if ( this->chainList.size() != 0 )  
			findBlobEstimate();
		else
			ROS_INFO("There are no previous blobs to match");

		/// for every blob, given as input to the tracker
		/// create a trackerChain to insert the blob
		createTrackerChains();

		/// if there are estimated blobs from previous
		/// blob occurrences, match input blobs with estimated blobs
		if (this->blobList->size() != 0) 
		{		
			matchmake();
#if TRACKER_DEBUG_MODE
			printInfo();
#endif
		}

	}

	/// check all trackerChains
	/// see which of them are inactive or active
	/// delete inactive chains , delete Things
	/// from overflown chains	
	checkTrackerChains();

	/// calculate probability of chain
	/// determine if each chain is consistent
	/// or is created because of noise
	calculateProbability();

	/// fill imageBuffer with the last Blobs
	/// of only 5 chains. This is done for
	/// debugging purposes
	fillImageBuffer();

	/// Process of estimating chains'
	/// new blob position. These Things have 
	/// frameId = -1, to be recognized
	for (int i=0; i<blobList->size(); i++)
	{
		Thing* tmp = blobList->at(i);
		if (tmp->frameId == -1)
		{
			tmp->destroyThing();
			delete tmp;
			tmp = NULL;
		}
	}
	/// after deleting estimated Things, clear blobList
	blobList->clear();
	currentList->clear();

	/// determine which blobs should be returned
	/// fill dst vector, with the last blob of
	/// every chain. Choose if you want to return
	/// all of them, or just the ones from the active chains
	for (int i=0; i<chainList.size(); i++)
	{
		TrackerChain* tc = chainList.at(i);
		Thing* current = tc->back();

		current->m_active = tc->active;		
		current->probability = tc->probability;

		/// send all blobs or only the active?
		if ( (current->m_active == false) && (SEND_ALL == false) )
			continue;

		dst->push_back( current );
	}

}

/**
 *  Calculate TrackerChains' probability
 * 	of holding real blobs. Probability is 
 *  calculated considering the blobs'
 * 	occurrences between frames
 */
void Tracker::calculateProbability()
{
	/// for every chain in chainList
	/// count how many blobs are younger
	/// than PROBABILITY_MAX_BLOB_AGE. 
	/// Divide this number with PROBABILITY_BLOB_NUM
	/// and get the probability
	for (int i=0; i<chainList.size(); i++)
	{
		double probability = 0;

		TrackerChain* tc = chainList.at(i);
		Thing* current = tc->back();

		for (int j=tc->size()-1; j>=0; j--)
		{
			Thing* tmp = tc->at(j);

			if (tmp->frameId <= frameNum - PROBABILITY_MAX_BLOB_AGE)
				break;

			probability++;	
		}

		probability = probability/PROBABILITY_BLOB_NUM;
		if (probability > 1)
			probability = 1;

		tc->probability = probability;
		current->probability = probability;
	}

}

/**
 * Estimate the position and size of the blob/hole, using prior knowledge.
 */
void Tracker::findBlobEstimate()
{	
	/// Fill blobList with estimated Things
	for (int i=0; i<chainList.size(); i++)
	{
		TrackerChain* tc = chainList.at(i);

		/// if trackerChain has less than 3 Blobs
		/// then don't estimate a blob, but send the 
		/// last blob of this chain as an estimation
		if (tc->size() < 3)
		{
			Thing* current = tc->back();
			blobList->push_back( current );
		}
		else 	
		{
			/// estimate Thing's center position, area and overlap. 
			/// use the last 3 Things for the calculations,
			/// if and only if all these 3 blobs are not 
			/// more than MAX_AGE_BLOB_ESTIMATE frames old
			Thing* first = tc->back();
			Thing* second = tc->at( tc->size() - 2 );
			Thing* third = tc->at( tc->size() - 3 );

			if (third->frameId < frameNum - MAX_AGE_BLOB_ESTIMATE)
			{
				Thing* current = tc->back();
				blobList->push_back( current );
				continue;
			}	

			/// create the new estimated Thing.
			/// set estimated Blobs' frameId to -1
			/// to be recognised later, as an 
			/// estimated Blob
			Thing* estimate = new Thing();
			estimate->createThing( first->m_imgBlob , first->m_contour);
			estimate->m_chainId = first->m_chainId;
			estimate->frameId = -1;

			//cout << "[Tracker] : Estimating " << estimate->m_id << " with " << first->m_id << endl;

			/// estimate next position of center's x coordinate
			CvPoint2D32f x1 = cvPoint2D32f( first->frameId , (int)first->center.x );
			CvPoint2D32f x2 = cvPoint2D32f( second->frameId , (int)second->center.x );
			CvPoint2D32f x3 = cvPoint2D32f( third->frameId , (int)third->center.x );

			double x = polyFit( x1 , x2 , x3 , CENTER_REGRESSION_ERROR);

			/// estimate next position of center's y coordinate
			CvPoint2D32f y1 = cvPoint2D32f( first->frameId , (int)first->center.y );
			CvPoint2D32f y2 = cvPoint2D32f( second->frameId , (int)second->center.y );
			CvPoint2D32f y3 = cvPoint2D32f( third->frameId , (int)third->center.y );

			double y = polyFit( y1 , y2 , y3 , CENTER_REGRESSION_ERROR);

			/// check if the center position is a valid one
			bool estimateCenter = true;
			if ( (x > 0) && (x < estimate->m_imgBlob->width) &&
					(y > 0) && (y < estimate->m_imgBlob->height) &&
					estimateCenter)
			{
				estimate->center.x = (x + 3*tc->back()->center.x)/4;;
				estimate->center.y = (y + 3*tc->back()->center.y)/4;
			}

			/// estimate area of blob
			//Don't estimate area... poor results
			/*
			CvPoint2D32f a1 = cvPoint2D32f( first->frameId , (int)first->area );
			CvPoint2D32f a2 = cvPoint2D32f( second->frameId , (int)second->area );
			CvPoint2D32f a3 = cvPoint2D32f( third->frameId , (int)third->area );

			double area = polyFit( a1 , a2 , a3 , AREA_REGRESSION_ERROR);
			double maxArea = estimate->m_imgBlob->width * estimate->m_imgBlob->height;

			if ( (area > 0) && (area < maxArea) )
			{
				//cout << "Area is " << area << endl;
				//cout << "Actual is " << estimate->area << endl;
				//cout << "da is " << da << endl;
				//cout << "s2 is " << s2 << endl;
				//cin.get();

				estimate->area = area;
			}
			 */


			/// estimate new imgBlob, for overlap calculation		
			///find roi, using Things' bounding rectangles
			cv::Mat imgBlobDilated = cv::Mat(first->m_imgBlob).clone();

			cv::dilate(imgBlobDilated,imgBlobDilated,cv::Mat(), cv::Point(), OVERLAP_DILATION_ITERATIONS);
			//cv::circle(imgBlobDilated, cv::Point(estimate->center.x,estimate->center.y), 60, cv::Scalar(255), -1, 8, 0);
#if TRACKER_DEBUG_MODE
			cv::imshow("111", imgBlobDilated);
#endif

			std::vector<std::vector<cv::Point> > contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::Mat contour_mat = imgBlobDilated.clone();
			findContours( contour_mat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

			cv::Rect originalRect = first->m_rect;
			cv::Rect dilatedRect = cv::boundingRect(contours[0]);
			cv::Rect newRect;
			newRect.width = dilatedRect.width;
			newRect.height = dilatedRect.height;
			newRect.x = estimate->center.x - (int)(newRect.width/2);
			newRect.y = estimate->center.y - (int)(newRect.height/2);	
			int dx = newRect.x - dilatedRect.x;
			int dy = newRect.y - dilatedRect.y;	

			//cout << "[Tracker] : originalRect " << originalRect.x << " " << originalRect.y << " " << originalRect.width << " " << originalRect.height << endl;
			//cout << "[Tracker] : dilatedRect " << dilatedRect.x << " " << dilatedRect.y << " " << dilatedRect.width << " " << dilatedRect.height << endl;
			//cout << "[Tracker] : newRect " << newRect.x << " " << newRect.y << " " << newRect.width << " " << newRect.height << endl;
			//cout << "[Tracker] : dx/dy " << dx << " " << dy << endl;

			/// copy pixels that lie inside the bounding rect 
			/// and their estimation lies in the image
			int startX = newRect.x;
			if (startX < 0)
				startX = 0;

			int endX = startX + newRect.width;
			if (endX > estimate->m_imgBlob->width)
				endX = estimate->m_imgBlob->width;

			int startY = newRect.y;
			if (startY < 0)
				startY = 0;

			int endY = startY + newRect.height;
			if (endY > estimate->m_imgBlob->height)
				endY = estimate->m_imgBlob->height;
			int step      = estimate->m_imgBlob->widthStep;
			int channels  = estimate->m_imgBlob->nChannels;
			uchar* data   = (uchar *)estimate->m_imgBlob->imageData;
			uchar* dataDil = imgBlobDilated.data;
			cvZero(estimate->m_imgBlob);

			//cout << "[Tracker] : Copy for startX: " << startX << " from " << startX-dx << endl;
			//cout << "[Tracker] : Copy for startY: " << startY << " from " << startY-dy << endl;

			for (int j=startX; j<endX; j++)
			{
				for (int i=startY; i<endY; i++)
				{					
					//if  ( (int)dataDil[(i-dy)*step+(j-dx)] == 255 )
					if  ( imgBlobDilated.at<uchar>((i-dy),(j-dx)) == 255 )
						data[i*step+j] = 255;
				}
			}
#if TRACKER_DEBUG_MODE
			cvShowImage("112" , estimate->m_imgBlob);
#endif
			contour_mat = cv::Mat(estimate->m_imgBlob).clone();
			findContours(contour_mat , contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
			estimate->m_rect = cv::boundingRect(contours[0]);

			//estimate->m_rect = cvBoundingRect(estimate->m_imgBlob);

			//cvReleaseImage(&imgBlobDilated);
			//cout << "areas are " << first->area << "||" << second->area << "||" << third->area << "||" << estimate->area << endl;

			blobList->push_back( estimate );
		}

	}

}

/**
 * Fitting function
 */ 
double Tracker::polyFit( CvPoint2D32f p1, CvPoint2D32f p2, CvPoint2D32f p3, double err )
{
	int n = 3;
	double xi;
	double yi;
	double ei;
	double chisq;

	CvPoint2D32f points[3];
	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	gsl_matrix *X, *cov;
	gsl_vector *y, *w, *c;

	X = gsl_matrix_alloc (n, 3);
	y = gsl_vector_alloc (n);
	w = gsl_vector_alloc (n);

	c = gsl_vector_alloc (3);
	cov = gsl_matrix_alloc (3, 3);

	for (int i = 0; i < n; i++)
	{
		xi = points[i].x;
		yi = points[i].y;
		ei = err;

		gsl_matrix_set (X, i, 0, 1.0);
		gsl_matrix_set (X, i, 1, xi);
		gsl_matrix_set (X, i, 2, xi*xi);

		gsl_vector_set (y, i, yi);
		gsl_vector_set (w, i, 1.0/(ei*ei));
	}

	gsl_multifit_linear_workspace * work = gsl_multifit_linear_alloc (n, 3);
	gsl_multifit_wlinear (X, w, y, c, cov, &chisq, work);
	gsl_multifit_linear_free (work);

#define C(i) (gsl_vector_get(c,(i)))
				//#define COV(i,j) (gsl_matrix_get(cov,(i),(j)))

	//printf ("# best fit: Y = %g + %g X + %g X^2\n",
	//		C(0), C(1), C(2));
	//printf ("# covariance matrix:\n");
	//printf ("[ %+.5e, %+.5e, %+.5e  \n",
	//		COV(0,0), COV(0,1), COV(0,2));
	//printf ("  %+.5e, %+.5e, %+.5e  \n",
	//		COV(1,0), COV(1,1), COV(1,2));
	//printf ("  %+.5e, %+.5e, %+.5e ]\n",
	//		COV(2,0), COV(2,1), COV(2,2));
	//printf ("# chisq = %g\n", chisq);

	double result = C(0) + C(1)*frameNum + C(2)*frameNum*frameNum;

	gsl_matrix_free (X);
	gsl_vector_free (y);
	gsl_vector_free (w);
	gsl_vector_free (c);
	gsl_matrix_free (cov);

	return result;
}

/**
 * Create TrackerChains for all blobs found at the current frame.
 * Add every one of these blobs to each own chain
 */ 
void Tracker::createTrackerChains()
{
	int size = this->currentList->size();
	for (int i=0; i<size ; i++)
	{
		/// create chain
		TrackerChain* tc = new TrackerChain();

		/// add blob to chain
		tc->push( currentList->at(i) );

		/// add chain to chainList
		chainList.push_back(tc);

#if TRACKER_DEBUG_MODE
		ROS_INFO("[Tracker] : added Thing with id : %d ",currentList->at(i)->m_id); 
#endif
	}
}

/**
 * Checks trackerchain for inactivity, overflow etc and disables or keeps it accordingly
 */
void Tracker::checkTrackerChains()
{	
	/// for each TrackerChain in chainList check in which
	/// frame, the last blob of the chain was observed. 
	/// Also, check if the trackerChain has overflown with Blobs.  
	/// If the last blob is N frames old, then 
	/// set chain's inactivity to N.
	/// If chain's inactivity is above a certain
	/// threshold, delete all blobs of the current
	/// chain and then delete the chain
	/// If the chain has more than K blobs, delete 
	for (int i=chainList.size()-1; i>=0; i--)
	{		
		TrackerChain* tc = chainList.at(i);
		Thing* current = tc->back();

		/// update chain's inactivity
		int inactivity = frameNum - current->frameId;
		tc->inactivity = inactivity;

		if ( inactivity > MAX_INACTIVITY )tc->inactive = true;

		/// update chain's activity
		if ( inactivity <= MIN_INACTIVITY )tc->active = true;
		else tc->active = false;

		//debug block
		{ 
			//cout << frameNum << " " << current->frameId << endl;
			//cout << "inactivity is " << inactivity << " for chain " << tc->m_id << endl;
			//cout << "delete inactive chains, size of chainList is : " << chainList.size() << endl;
			//cin.get();
		}

		/// if inactivity is above MAX_INACTIVITY
		/// delete all blobs in the chain, and then
		/// delete the chain itself
		if (tc->inactive == true)
		{
			//debug block
			{
				ROS_INFO("!!!!  Found that trackerchain %d is inactive!", tc->m_id );
				ROS_INFO( "%d %d ",frameNum,current->frameId );
				//cin.get();
			}

			/// erase this chain from chainList
			chainList.erase( chainList.begin() + i );

			/// destroy all containing things
			for (int j=0; j<tc->size(); j++)
			{
				Thing* current = tc->at(j);
				current->destroyThing();
				delete current;
				current = NULL;
			}

			/// clear all things from vector
			tc->clear();

			/// delete trackerChain
			delete tc;
			tc = NULL;
		}

		/// delete Things from overflown chains
		/// A chain is overflown, if it contains
		/// more blobs than MAX_CHAIN_BUFFER
		if (tc)
		{
			if (tc->size() > MAX_CHAIN_BUFFER) 
			{
				int pos = tc->size() - MAX_CHAIN_BUFFER;

				/// destroy all Things, that are not stored in 
				/// the last MAX_CHAIN_BUFFER positions 
				for (int j = 0; j < pos; j++)
				{
					Thing* current = tc->at(j);
					current->destroyThing();
					delete current;
					current = NULL;
				}

				/// erase all deleted Things from the chain
				tc->erase( tc->begin() , tc->end() - MAX_CHAIN_BUFFER );
			}
		}

	}

}

/**
 * Matches a given blob with every blob of enabled trackerchains.
 * If we have a match, that blob becomes part of the relevant trackerchain.
 */
void Tracker::matchmake()
{	
	///////////////////////////////////////
	//
	//Create score matrix
	//
	///////////////////////////////////////

	/// List previousBlobs contains blobs from previous frames
	/// List currentBlobs contains blobs from current frame

	std::vector<Thing*>* previousBlobs = this->blobList;
	std::vector<Thing*>* currentBlobs = this->currentList;

	int rows = previousBlobs->size();
	int cols = currentBlobs->size();
	float scoreMatrix[rows][cols];		//holds scores for matching blobs

	std::vector<int> indexMaxPerRow;			//hold position(column) of maximum scoring previousBlobs for each currentBlob
	std::vector<int> indexMaxPerColumn;		//hold position(row) of maximum scoring currentBlob for each previousBlobs

	// Calculate Score and fill score Matrix
	for (int i=0; i<rows; i++)
	{
		for (int j=0; j<cols; j++)
		{
			scoreMatrix[i][j] = calculateScore( previousBlobs->at(i), currentBlobs->at(j) );
		}
	}

	//print score matrix
	//cout << endl << endl << "--MATCHING TAKES PLACE" << endl;
	//cout << "Score Matrix is " << endl;
#if TRACKER_DEBUG_MODE
	for (int i=0; i<rows; i++)
	{
		for (int j=0; j<cols; j++)
		{			
			ROS_INFO("""%d [ %d "," %d ] ", scoreMatrix[i][j],previousBlobs->at(i)->m_id,currentBlobs->at(j)->m_id;
		}
	}
#endif
	// Find best matches in currentBlobs list
	for (int i=0; i<rows; i++)
	{
		double max = scoreMatrix[i][0];
		int pos = 0;
		for (int j=1; j<cols; j++)
		{
			if (scoreMatrix[i][j] > max)
			{
				max = scoreMatrix[i][j];
				pos = j;
			}
		}
		indexMaxPerRow.push_back(pos);
	}

	// Find best matches in blobList blobs
	for (int j=0; j<cols; j++)
	{
		double max = scoreMatrix[0][j];
		int pos = 0;
		for (int i=1; i<rows; i++)
		{
			if (scoreMatrix[i][j] > max)
			{
				max = scoreMatrix[i][j];
				pos = i;
			}
		}
		indexMaxPerColumn.push_back(pos);
	}

	// Find the matches
	for (int i=0; i<rows; i++)
	{
		for (int j=0; j<cols; j++)
		{

			// Check if there are any matched Blobs
			// If matches are found, move the current matched blob
			// to the corresponding chain

			if ( (indexMaxPerRow.at(i) == j) && (indexMaxPerColumn.at(j) == i) && (scoreMatrix[i][j] > 0) )
			{
#if TRACKER_DEBUG_MODE
				ROS_INFO("Blob matched!"); 
				ROS_INFO("indexes are currentBlobs at i : %d blobList at j : %d" ,i,j);
				ROS_INFO("score is %f",scoreMatrix[i][j]); 
#endif
				//cin.get();

				//cout << "previousBlobs size is " << previousBlobs->size() << endl;
				Thing* prevThing = previousBlobs->at(i);
				//cout << "currentBlobs size is " << currentBlobs->size() << endl;
				Thing* currThing = currentBlobs->at(j);

				//only for visualization purposes
				currThing->matched_center = prevThing->center;

				TrackerChain* prevTc = (TrackerChain*)prevThing->m_chainId;
				TrackerChain* currTc = (TrackerChain*)currThing->m_chainId;

				prevTc->push(currThing);

				// pop currTc from chainList
				for (int i=chainList.size()-1; i>=0; i--)
				{
					if (chainList.at(i)->m_id == currTc->m_id)
						chainList.erase( chainList.begin() + i );
				}

				// currTc is now useless. Delete it
				currTc->clear();
				delete currTc;
			}

		}
	}

}

/**
 * Calculates the score of two things, in order to match them.
 * 
 * Score will be calculated, following these rules :
 *	 - Higher score for blobs, with minimum distance between frames
 *	 - Higher score for blobs, with almost the same area
 *	 - Higher score for blobs, with high overlap ratio
 *	
 *		Score for distance : 
 *			d = (xA - xB)^2 + (yA - yB)^2		//distance
 *			s1 = exp( -L1 * d )					//score for distance
 *	
 *		Score for area : 
 * 			a = abs(areaB - areaA) / sqrt(areaA + areaB)		
 *			s2 = exp( -L2 * a)					//score for area
 *	
 *		Score for overlap :
 *			o = min { commonPixels / areaB , commonPixels/areaA }
 *			s3 = ( 1 - exp( -L3 * o) ) / (1 - exp(-L3) )		//score for overlap
 * 
 */ 
double Tracker::calculateScore(Thing* a , Thing* b)
{

	//cout << "Calculating score for " << a->m_id << " " << b->m_id << endl;

	///////////////////////////////////////////
	//
	/// calculate score from center distance
	//
	///////////////////////////////////////////
	double dx = pow( (a->center.x - b->center.x) , 2);
	double dy = pow( (a->center.y - b->center.y) , 2);
	double d = dx + dy;
	double s1 = exp( -L1 * d );

	//cout << "s1 " << s1 << endl;

	///////////////////////////////////////////
	//
	/// calculate score from area difference
	//
	///////////////////////////////////////////
	double da;
	double s2;
	if (a->area != 0)
	{
		da = 2 * abs(b->area - a->area)/(a->area + b->area);
		s2 = exp( -L2 * da );
	}
	else
		ROS_INFO("Area of Blob is ZERO!!!");

	//cout << "s2 " << s2 << endl;

	///////////////////////////////////
	//
	/// calculate score from overlap
	//
	///////////////////////////////////
	IplImage* imgOverlap = (IplImage*)cvClone(a->m_imgBlob);
	cvAnd(a->m_imgBlob , b->m_imgBlob , imgOverlap , NULL);

	//cout << "Find roi for overlap" << endl;
	//find roi, using things' bounding rectangles
	CvRect temp = a->m_rect;

	if (b->m_rect.x < temp.x)
		temp.x = b->m_rect.x;

	if (b->m_rect.y < temp.y)
		temp.y = b->m_rect.y;

	if (b->m_rect.width > temp.width)
		temp.width = b->m_rect.width;

	if (b->m_rect.height > temp.height)
		temp.height = b->m_rect.height;

	//search only inside the bounding rect
	int startX = temp.x;
	int endX = startX + temp.width;
	int startY = temp.y;
	int endY = startY + temp.height;

	//cout << "startX is " << startX << endl;
	//cout << "endX is " << endX << endl;
	//cout << "startY is " << startY << endl;
	//cout << "endY is " << endY << endl;

	int step      = imgOverlap->widthStep;
	int channels  = imgOverlap->nChannels;
	uchar* data   = (uchar *)imgOverlap->imageData;

	int commonPixels = 0;
	for (int j=startX; j<endX; j++)
	{
		for (int i=startY; i<endY; i++)
		{					
			if  ( (int)data[i*step+j] == 255 ) 		
				commonPixels++;		
		}
	}	
	cvReleaseImage(&imgOverlap);

	double overlap;
	overlap = commonPixels / std::max(a->area , b->area);

	double s3 = ( 1 - exp( -L3 * overlap) ) / (1 - exp(-L3) );


	//cout << "!!!!!! FrameNum : " << frameNum << endl;
	//cout << "!!!!!! Area a : " << a->area << endl;
	//cout << "!!!!!! Area b : " << b->area << endl;
	//cout << "!!!!!! Ratio is da : " << da << endl;
	//cout << "!!!!!! Overlap is : " << overlap << endl;
	//cout << "!!!!!! Score is s1 : " << s1 << endl;
	//cout << "!!!!!! Score is s2 : " << s2 << endl;
	//cout << "!!!!!! Score is s3 : " << s3 << endl;

	if ( 	((s1 < THRESHOLD) && (W1 > 0)) || 
			((s2 < THRESHOLD) && (W2 > 0)) || 
			((s3 < THRESHOLD) && (W3 > 0)) 
	)
		return 0;		// not sure about it...

	double score = (W1*s1 + W2*s2 + W3*s3) / (W1 + W2 + W3);		

	if (score < THRESHOLD)
		return 0;

	return score;
}

/**
 * Prints debugging info to the console.
 */
void Tracker::printInfo()
{
	ROS_INFO("chainList size is %d",chainList.size());

	for (int i=0; i<chainList.size(); i++)
	{
		ROS_INFO("Now checking trackerchain with id %d",chainList.at(i)->m_id); 
		for (int j=0; j<chainList.at(i)->size(); j++)
		{
			ROS_INFO("Thing in trackerChain frameCount :%d",chainList.at(i)->at(j)->frameId);
			ROS_INFO("Thing in trackerChain id : %d", chainList.at(i)->at(j)->m_id);
		}
	}
}

/**
 * Fills the image buffer of a trackerchain
 */
void Tracker::fillImageBuffer()
{
	for (int i=0; i<5; i++)
	{
		cvZero(imageBuffer[i]);
	}
	std::vector<std::vector<cv::Point> >draw_vector(1);
	for (int i=0; i<chainList.size(); i++)
	{
		if (i<5)
		{
			TrackerChain* tc = chainList.at(i);
			Thing* current = tc->back();

			int blue = (int)tc->color.val[0]*tc->probability;
			int green = (int)tc->color.val[1]*tc->probability;
			int red = (int)tc->color.val[2]*tc->probability;
			CvScalar newColor = cvScalar( blue , green , red );
			//cvDrawContours( imageBuffer[i], (CvSeq*)current->m_contour, newColor, cvScalarAll(0), -1, CV_FILLED, 8, cvPoint(0,0));

			cv::Mat local2 = cv::Mat(imageBuffer[i]);
			//std::cout<< "[Tracker] : before"<<std::endl;

			draw_vector.at(0) = current->m_contour;
			cv::drawContours(local2,draw_vector,-1,newColor,CV_FILLED);
			//std::cout<< "[Tracker] : after"<<std::endl;

			// Draw circles to the matched blob centers and then connect them with a line
			CvPoint c1 = cvPoint( (int)current->center.x , (int)current->center.y );
			CvPoint c2 = cvPoint( (int)current->matched_center.x , (int)current->matched_center.y );

			cvCircle(imageBuffer[i], c1 , 6, cvScalar(0,0,255), 2, 8, 0);
			cvCircle(imageBuffer[i], c2 , 6, cvScalar(0,0,255), 2, 8, 0);

			//cout << "Probability is " << tc->probability << endl;
			//memcpy(imageBuffer[i]->imageData , current->m_imgBlob->imageData , imageBuffer[i]->imageSize);
			//cvCircle(imageBuffer[i], cvPoint(320,240), 50, tc->color, 4, 8, 0);
		}
	}
}

/**
 * CamShift tracking algorithm, for tracking a visible hole (the custom tracker allowed the hole
 * to stay off frame for a short period of time)
 * @param frame
 * @param imgTexture
 * @param blobs
 * @return
 */
IplImage* Tracker::camShiftTrack(IplImage* frame, IplImage* imgTexture, std::vector<Thing*>* blobs){

	if(camShiftBlobs.size() == 0)
	{
		for(int i=0; i<blobs->size() ; i++)
		{
			CamShiftThing newHole;
			newHole.id = blobs->at(i)->m_chainId;
			newHole.trackRect = blobs->at(i)->m_rect;
			newHole.area = blobs->at(i)->area;
			camShiftBlobs.push_back(newHole);
		}
	}

	bool isCamShiftTracked = false;

	for(int i=0 ; i<blobs->size() ; i++)
	{
		for(int j=0 ; j<camShiftBlobs.size() ; j++)
		{
			if( blobs->at(i)->m_chainId == camShiftBlobs[j].id){
				isCamShiftTracked = true;
				break;
			}
		}

		if(!isCamShiftTracked){
			CamShiftThing newHole;
			newHole.id = blobs->at(i)->m_chainId;
			newHole.trackRect = blobs->at(i)->m_rect;
			newHole.area = blobs->at(i)->area;
			camShiftBlobs.push_back(newHole);
		}

		//blob is pushed in vector, reset the flag:
		isCamShiftTracked = false;
	}

	if (camShiftBlobs.size() == 0){
		cvZero(csImage);
		cvZero(csAnd);

		return csImage;
	}

	int vmin = this->csVmin;
	int vmax = this->csVmax; 
	int smin = this->csSmin;
	int smax = this->csSmax;

	int hdims = 16;
	float hranges_arr[] = {0,180};
	float* hranges = hranges_arr;

	CvHistogram* csHist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );

	CvConnectedComp* csTrackComp = new CvConnectedComp;
	CvBox2D* csTrackBox = new CvBox2D;

	//cvCopy( frame, csImage, 0 );
	cvCvtColor( frame, csHsv, CV_BGR2HSV );

	cvInRangeS( csHsv, cvScalar(0, MIN(smin,smax), MIN(vmin,vmax), 0),
			cvScalar(180, MAX(smin,smax), MAX(vmin,vmax), 0) , csMask );
	cvSplit( csHsv, csHue, 0, 0, 0 );

	for(int i=0 ; i<camShiftBlobs.size() ; i++){
		cvSetImageROI( csHue, camShiftBlobs[i].trackRect );
		cvSetImageROI( csMask, camShiftBlobs[i].trackRect );
		cvCalcHist( &csHue, csHist, 0, csMask );
		cvResetImageROI( csHue );
		cvResetImageROI( csMask );

		cvCalcBackProject( &csHue, csBackproject, csHist );
		cvAnd( csBackproject, csMask, csBackproject, 0 );

		cvCamShift( csBackproject, camShiftBlobs[i].trackRect ,
				cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
				csTrackComp, csTrackBox );

		camShiftBlobs[i].trackRect = csTrackComp->rect;

		///TODO: Find a better way to calculate area, or search why csTrackComp.area is wrong
		camShiftBlobs[i].area = (double)(csTrackComp->rect.width * csTrackComp->rect.height);

		if(csTrackComp->rect.x < 0 || csTrackComp->rect.x > frame->width){
			camShiftBlobs[i].probability = 0.;
			continue;
		}
		if(csTrackComp->rect.y < 0 || csTrackComp->rect.y > frame->height){
			camShiftBlobs[i].probability = 0.;
			continue;
		}
		if(csTrackComp->rect.width < 0 || csTrackComp->rect.width > frame->width){
			camShiftBlobs[i].probability = 0.;
			continue;
		}
		if(csTrackComp->rect.width < 0 || csTrackComp->rect.width > frame->width){
			camShiftBlobs[i].probability = 0.;
			continue;
		}

		//calculate probability
		cvZero(csAnd);
		cvRectangle(csAnd, cvPoint(csTrackComp->rect.x,csTrackComp->rect.y) , cvPoint(csTrackComp->rect.x + csTrackComp->rect.width,csTrackComp->rect.y + csTrackComp->rect.height), cvScalarAll(255), CV_FILLED, 8, 0);
		double ellipseArea = cvNorm( csAnd , 0, CV_L1, 0 )/255. ;
		//printf("CamshiftBlob %d ellipse area is: %f", i, ellipseArea);
		cvAnd(csAnd , imgTexture , csAnd);
		if(ellipseArea > 0)
			camShiftBlobs[i].probability = (double)(cvNorm( csAnd , 0, CV_L1, 0 )/255. / ellipseArea);
		else
			camShiftBlobs[i].probability = 0.;
	}

	for(int i=camShiftBlobs.size()-1 ; i>=0 ; i--){
		//check if tracking needs to stop
		bool rectIsTooBig =( 	( camShiftBlobs[i].trackRect.width > (int)(0.88*frame->width) ) || 
				( camShiftBlobs[i].trackRect.height > (int)(0.88*frame->height) ) 	);

		bool rectIsTooSmall =(	( camShiftBlobs[i].trackRect.width < 30 ) ||
				( camShiftBlobs[i].trackRect.height < 30 ) 	);

		bool rectIsWayTooFar =( ( camShiftBlobs[i].trackRect.x < 20 ) ||
				( camShiftBlobs[i].trackRect.y < 20 ) ||
				( ( camShiftBlobs[i].trackRect.x + camShiftBlobs[i].trackRect.width) > (frame->width - 20) ) ||
				( ( camShiftBlobs[i].trackRect.y + camShiftBlobs[i].trackRect.height) > (frame->height - 20) )	);

		bool rectMayNotBeHole = ( ( camShiftBlobs[i].probability < 0.40 ) );

		if(rectIsTooBig || rectIsTooSmall || rectIsWayTooFar || rectMayNotBeHole){
			camShiftBlobs.erase( camShiftBlobs.begin() + i );
		}
	}


	//debugging -->
	cvCopy(frame, csImage);	

	for(int i=0 ; i<camShiftBlobs.size() ; i++)
	{		
		int x = camShiftBlobs[i].trackRect.x;
		int y = camShiftBlobs[i].trackRect.y;
		int width = camShiftBlobs[i].trackRect.width;
		int height = camShiftBlobs[i].trackRect.height;
		cvRectangle( csImage, cvPoint( x, y ) , cvPoint( x+width, y+height ) , cvScalar(255, 0, 0) , 3, 8);
	}
	//<--

	/*
	if(camShiftBlobs.size() > 10){
		camShiftBlobs.clear();
	}*/

	delete csTrackComp;
	delete csTrackBox;

	cvReleaseHist(&csHist);

	return csImage;
}
}

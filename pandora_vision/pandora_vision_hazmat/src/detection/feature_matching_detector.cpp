/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Choutas Vassilis 
 *********************************************************************/


#include "pandora_vision_hazmat/detection/feature_matching_detector.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {

    /*
     * @brief: Main constructor for the feature based detectors that reads
     * all the necessary training data.
     * @param featureName[const std::string& ]: The name of the features that
     * will be used.
     */
    FeatureMatchingDetector::FeatureMatchingDetector(
        const std::string &featureName)
      :PlanarObjectDetector(featureName)
    {
      // Read the necessary data.
      bool readFlag = this->readData();
      if (!readFlag)
      {
        ROS_FATAL_STREAM("[Hazmat Detection]: Could not read the necessary" 
                         << " training data!");
        ROS_FATAL_STREAM("[Hazmat Detection]: The node will now shutdown!");
        ROS_BREAK();
      }
      ROS_INFO_STREAM("[Hazmat Detection]: Succesfully read training Data!");
    }

    /**
      @brief Function used to read the necessary training data for
      the detector to function.
      @return [bool] : A flag that tells us whether we succeeded in 
      reading the data.
     **/

    bool FeatureMatchingDetector::readData(void)
    {
      // Open the file for reading .
      std::string patternNameInputFile = fileName_ + "/data/input.xml";
      cv::FileStorage fs( patternNameInputFile , cv::FileStorage::READ);
      // Check if the file was opened succesfully .
      if ( !fs.isOpened() )
      {
        ROS_ERROR_STREAM("[Hazmat Detection]: XML file for training data" 
                         <<" file names could not be opened!");
        return false;
      }

      // Go to the xml node that contains the pattern names.
      cv::FileNode inputNames = fs["PatternName"];
      if (inputNames.empty())
      {
        ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]: The xml" 
                               << " file does not contain a node named :"
                               << " PatternName!");
        return false;
      }
      // Check if the node has a sequence.
      if ( inputNames.type() != cv::FileNode::SEQ)
      {
        ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]: Input data" 
                               << " is not a string sequence!");
        return false;
      }

      // Initialize File iterator.
      cv::FileNodeIterator it = inputNames.begin();
      cv::FileNodeIterator itEnd = inputNames.end();
      std::string inputName;
      std::vector<std::string> input;

      // Iterate over the node and get the names.
      for ( ; it != itEnd ; ++it ) 
      {
        if ( (*it)["name"].empty() )
        {
          ROS_DEBUG_STREAM_NAMED("detection", "[Hazmat Detection]: No name" 
                                 << " tag exists for the current value of the "
                                 << "iteration. The procedure will continue" 
                                 << " with the next node!");
          continue;
        }
        inputName = (std::string)(*it)["name"];
        input.push_back(inputName);
      }

      // Check if the names of the patters where read successfully.
      if (input.size() <= 0)
      {
        ROS_FATAL_STREAM("No patterns names were read from the "
                         << " provided file!");
        return false;
      }
      // Close the file with the pattern names.
      fs.release();

      // For every pattern name read the necessary training data.
      std::string trainingDataDir = fileName_ + std::string( "/data/"
          "training_data/")
        + this->getFeaturesName();

      std::string fileName;

      for (int i = 0 ; i < input.size() ; i++) 
      { 
        // Open the training file associated with image #i .
        fileName = trainingDataDir + "/" + input[i] + ".xml";
        cv::FileStorage fs2( fileName.c_str() , cv::FileStorage::READ);

        // Check if the file was properly opened.
        if ( !fs2.isOpened() )
        {
          ROS_WARN_STREAM_NAMED("detection", "[Hazmat Detection]: File "
                                 << fileName << " could not be opened!");
          continue;
        }

        std::vector<cv::Point2f> keyPoints;
        std::vector<cv::Point2f> boundingBox;
        cv::Mat descriptors; 

        // Read the pattern's descriptors.
        if ( fs2["Descriptors"].empty() )
        {
          ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]: No " 
                                 << "descriptors entry was specified for" 
                                 << " the pattern : " <<  fileName);
          continue;
        }
        fs2["Descriptors"] >> descriptors;

        // Read the pattern's keypoints.
        if ( fs2["PatternKeypoints"].empty() )
        {
          ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]: No " 
              << "Keypoints entry was specified for "
              <<  " the pattern : " << fileName);
          continue;
        }
        cv::FileNode keyPointsNode = fs2["PatternKeypoints"];

        // Initialize node iterators.
        cv::FileNodeIterator it = keyPointsNode.begin();
        cv::FileNodeIterator itEnd = keyPointsNode.end();

        cv::Point2f tempPoint;
        int counter = 0;
        // Iterate over the node to get the keypoints.
        for ( ; it != itEnd ; ++it ) 
        {
          if ( (*it)["Keypoint"].empty() )
          {
            ROS_WARN_STREAM_NAMED("detection", "[Hazmat Detection]: Error when"
                                   << " reading keypoint " << counter + 1 
                                   << " for pattern " << fileName << "!");
            continue;
          }
          (*it)["Keypoint"] >> tempPoint;
          keyPoints.push_back(tempPoint);
          counter++;
        }
        if (keyPoints.size() <= 0)
        {
          ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]: No"
                                 << "keypoints were read from the "
                                 << " specified file for "
                                 << "the pattern " << fileName << " !");
          continue;
        }
        // Read the pattern's bounding box. 
        cv::FileNode boundingBoxNode = fs2["BoundingBox"];
        if (boundingBoxNode.empty() )
        {
          ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]: No bounding"
                                 << " box entry exists for the pattern "  
                                 << fileName << " !");
          continue;
        }
        // Initialize it's iterator.
        cv::FileNodeIterator bbIt = boundingBoxNode.begin();
        cv::FileNodeIterator bbItEnd = boundingBoxNode.end();


        for ( ; bbIt != bbItEnd ; ++bbIt ) 
        {
          (*bbIt)["Corner"] >> tempPoint;
          boundingBox.push_back(tempPoint);
        }
        if (boundingBoxNode.size() <= 0 )
        {
          ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]: The" 
                                 << " bounding box for the pattern"
                                 << fileName << " could not be read!");
          continue;
        }
        if (boundingBoxNode.size() < 5 )
        {
          ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]: Not all "
                                 << "the points of the bounding box for" 
                                 << " the pattern " << fileName
                                 << "were read!");
          continue;
        }
        // Add the pattern to the pattern vector.
        Pattern p;
        p.name = input[i];
        p.boundingBox = boundingBox;
        p.keyPoints = keyPoints;
        p.descriptors = descriptors;
        // p.histogram = histogram;
        patterns_->push_back(p);

        // Close the xml file .
        fs2.release();
      }

      if (patterns_->size() > 0 )
      {
        if (patterns_->size() != input.size())
          ROS_WARN_STREAM_NAMED("detection", "[Hazmat Detection]: Could not" 
                                 << " read the training files for all "
                                 "the patterns!\n");
        return true;
      }
      else
      {
        ROS_FATAL_STREAM_NAMED("detection", "[Hazmat Detection]: No pattern "
                               << " was read! The node cannot function!");
        ROS_BREAK();
      }
      return true;
    }


    /**
      @brief Function used to detect matches between a pattern and the 
      input frame.
      @param frameDescriptors [const cv::Mat & ] : Descriptors of the frame.
      @param patternDescriptors [const cv::Mat &] : Descriptors of the 
      pattern.
      @param patternKeyPoints [std::vector<cv::KeyPoint> *] : Vector of 
      detected keypoints in the pattern.
      @param sceneKeyPoints [std::vector<cv::KeyPoint> *] : Vector of 
      detected keypoints in the frame.
     **/


    bool FeatureMatchingDetector::findKeypointMatches(
        const cv::Mat& frameDescriptors ,
        const cv::Mat& patternDescriptors , 
        const std::vector<cv::Point2f>& patternKeyPoints ,
        const std::vector<cv::KeyPoint>& sceneKeyPoints ,
        std::vector<cv::Point2f>* matchedPatternKeyPoints , 
        std::vector<cv::Point2f>* matchedSceneKeyPoints , 
        const int &patternID) 
    {
      // Clear the vectors containing the matched keypoints.
      matchedSceneKeyPoints->clear();
      matchedPatternKeyPoints->clear();

      // Define the vector of vectors that contains the matches.
      // Each element is a vector that contains the first,the second
      // up to the n-th best match.
      std::vector< std::vector<cv::DMatch> > matches;
      // Check if the keypoints for pattern #patternID have been loaded.
      if (patternKeyPoints.size() <= 0)
      {
        ROS_WARN_STREAM_NAMED("detection", "[Hazmat Detection]: No keypoints"
                              " stored for pattern " << patternID);
        return false;
      }

      // Check if we have stored the descriptors for pattern #patternID.
      if (patternDescriptors.data == NULL)
      {
        ROS_WARN_STREAM_NAMED("detection", "[Hazmat Detection]: No descriptors"
                              " stored for pattern " << patternID);
        return false;
      }
      // No keypoints detected in the scene so the matching cannot continue.
      if (sceneKeyPoints.size() <=0 )
      {
        ROS_WARN_STREAM_NAMED("detection", "[Hazmat Detection]: No keypoints"
                              <<  " were detected in the current scene!");
        return false;
      }
      // No descriptors calculated for the current frame.
      if (frameDescriptors.data == NULL)
      {
        ROS_WARN_STREAM_NAMED("detection", "[Hazmat Detection]: No "
                              << "descriptors were calculated in the current" 
                              << " scene!");
        return false;
      }


      // Perfom the matching using the matcher of the patternID-th 
      // pattern and find the top 2 correspondences. 
      matchers_[patternID]->knnMatch(frameDescriptors , matches , 2 );

      // The vector containing the best matches
      std::vector< cv::DMatch > goodMatches;

      // If we have found any matches.
      if ( matches.size() > 0 )
      {

        // We filter that matches by keeping only those
        // whose distance ration between the first and the second
        // best match is below a certain threshold.
        // TO DO : READ THE THRESHOLD FROM FILE.

        float ratio = 0.6;

        for( int i = 0; i <  matches.size() ; i++ )
        { 
          if( matches[i][0].distance < ratio*matches[i][1].distance )
          { 
            goodMatches.push_back( matches[i][0]); 
          }
        }

      }
      // No matches found.
      else
        return false;
      // Add the keypoints of the matches found to the corresponding
      // vectors for the pattern and the scene.
      for( int i = 0; i < goodMatches.size(); i++ )
      { 
        // Pattern key points .
        matchedPatternKeyPoints->push_back( 
            patternKeyPoints[ goodMatches[i].trainIdx ] );

        // Scene key points .
        matchedSceneKeyPoints->push_back( 
            sceneKeyPoints[ goodMatches[i].queryIdx ].pt );
      }

      // If we have less than 4 matches then we cannot find the Homography
      // and this is an invalid pattern.

      if ( goodMatches.size() < 4 )
        return false;

      return true;
    }

} // namespace pandora_vision_hazmat
} // namespace pandora_vision

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

#include <map>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

#include "pandora_vision_hazmat/training/planar_pattern_trainer.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /**
     * @brief Main training function that reads input images and stores
     * the results in a corresponding xml file.
     * @return bool : Flags that indicates success.
     */
    bool PlanarPatternTrainer::train()
    {
      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Starting Training!");
      const clock_t startingTime = clock();

      // The number of images.
      int imgNum;

      // The temporary container for the images.
      cv::Mat img;

      // The vector of the training image set.
      std::vector<cv::Mat> images;
      std::string imgName;
      std::string inputFile;

      std::vector<std::string> inputDataNames;

      // Open the file with the different pattern names.
      std::string fileName;
      std::string line;

      // ROS_INFO("Reading the names of the patterns.");

      // Initialize the path to the data that will be used to train the system.
      boost::filesystem::path trainingInputPath(packagePath_ +
          "/data/training_input/");

      // Create the container for all the files in the directory containing
      // the input of the training module.
      std::vector<boost::filesystem::path> trainingInputContents;

      // Check if the provided path exists.
      if (!boost::filesystem::exists(trainingInputPath))
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : The directory %s that"
            " contains the training input data is not a valid path!",
            trainingInputPath.c_str());
        return false;
      }

      // Iterate over the provided directory and store all
      // the files/paths it contains.
      copy(boost::filesystem::directory_iterator(trainingInputPath),
          boost::filesystem::directory_iterator(),
          std::back_inserter(trainingInputContents));

      // Sort the resulting data.
      sort(trainingInputContents.begin(), trainingInputContents.end());

      bool iterationFlag = false;
      // Iterate over all the files/paths in the subdirectory.
      for (std::vector<boost::filesystem::path>::iterator
          dirIterator(trainingInputContents.begin());
          dirIterator != trainingInputContents.end(); dirIterator++)
      {
        ROS_INFO("%s", dirIterator->c_str());
        try
        {
          // Check if the provided path exists.
          if (boost::filesystem::exists(*dirIterator))
          {
            // Check if it is a file.
            if (boost::filesystem::is_regular_file(*dirIterator))
            {
              iterationFlag |= singleViewTraining(*dirIterator);
            }

            // Check if it is a directory.
            else if (boost::filesystem::is_directory(*dirIterator))
            {
              ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : %s is a "
                  "directory !", dirIterator->c_str());
              ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Iterating over "
                  "all the files in the directory!");

              iterationFlag |= directoryProcessor(*dirIterator);
            }
            else
            {
              ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Path %s exists "
                  "but is neither a file nor a directory!",
                  dirIterator->c_str());
              continue;
            }
          }
          else
          {
            ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Invalid path!");
            continue;
          }
        }
        catch(const boost::filesystem3::filesystem_error& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }

      const clock_t endingTime = clock();
      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Features calculated and saved.");
      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Training is over !");
      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Training time was : "
          "%f seconds", (endingTime - startingTime) /
          static_cast<double>(CLOCKS_PER_SEC));

      // If true then at least the training data for one pattern was acquired
      // successfully.
      return iterationFlag;
    }

    /**
     * @brief This method iterates over a directory, reads every
     * instance/synthetic view,calculates features and keypoints for every
     * single one of them and stores them in a corresponding xml file.
     * @param dirPath[const boost::filesystem::path&]: Path to the directory
     * containing the images.
     * @return bool : Flags that indicates success.
     */
    bool PlanarPatternTrainer::directoryProcessor(
        const boost::filesystem::path& dirPath)
    {
      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Processing directory : %s ",
          dirPath.c_str());
      std::vector<boost::filesystem::path> pathVector;

      // Iterate over the provided directory and store all
      // the paths it contains.
      copy(boost::filesystem::directory_iterator(dirPath),
          boost::filesystem::directory_iterator(),
          std::back_inserter(pathVector));

      // Sort the contained paths.
      sort(pathVector.begin(), pathVector.end());

      // Get the path to the image that represents the frontal view of
      // the pattern.
      std::vector<boost::filesystem::path>::iterator frontalViewPathIter;

      // Get the path to the frontal view.
      boost::filesystem::path frontalViewPath(
          dirPath.string() + "/" +
          dirPath.filename().string() + std::string(".png"));
      // Find the path in the container for the current subdirectory.
      frontalViewPathIter = std::find(pathVector.begin(),
          pathVector.end(), frontalViewPath);

      // Vector that contains the training set images.
      std::vector<cv::Mat> imageCollection;
      std::vector<std::string> imageNames;

      // Check if the path was found.
      if (frontalViewPathIter == pathVector.end() ||
          !boost::filesystem::exists(*frontalViewPathIter))
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Could not find the"
            " frontal view as specified by"
            " %s !", frontalViewPath.c_str());
        // Since the frontal view was not found,training cannot continue.
        return false;
      }
      // Check if it is a file.
      if (boost::filesystem::is_regular_file(*frontalViewPathIter))
      {
        cv::Mat img = cv::imread(frontalViewPathIter->c_str());

        if (!img.data)
        {
          ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Could not read "
              "image,proceeding to next pattern!");
          return false;
        }
        // Add the frontal view to the collection.
        imageCollection.push_back(img);
        imageNames.push_back(frontalViewPathIter->filename().
            stem().string());
      }
      else
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : The path : %s is not"
            " a valid path. Continuing with the"
            " next path!", frontalViewPathIter->c_str());
        return false;
      }

      // Remove the frontal view from the entries.
      int frontalViewPos = std::distance(pathVector.begin(),
          frontalViewPathIter);
      pathVector.erase(pathVector.begin() + frontalViewPos);

      // Get the path to the file containing the homographies that
      // created all the synthetic views.
      std::vector<boost::filesystem::path>::iterator
        homographyDataStoragePathIter;

      boost::filesystem::path homographyPath(dirPath.string() +
          std::string("/homography.txt"));

      homographyDataStoragePathIter = std::find(pathVector.begin(),
          pathVector.end(), homographyPath);

      // Initialize the container for the homographies.
      std::map<std::string, cv::Mat> homographies;

      // Check that the provided file is indeed a file.
      if (boost::filesystem::is_regular_file(homographyPath))
      {
        // Get the homography matrices.
        bool flag = getHomographyMatrices(homographyPath, &homographies);
        // If we failed to read the homographies then the training cannot
        // continue.
        if (!flag)
          return false;
      }
      else
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Could not find the "
            "file containing the homography "
            "transformations that generated the new views!");
        return false;
      }
      // Find the position of the path of the homography input file
      // in the list of paths.
      int homographyDataStoragePathPos = std::distance(pathVector.begin(),
          homographyDataStoragePathIter);

      // Delete the entry of the homography matrix from the list of
      // paths.
      pathVector.erase(pathVector.begin() + homographyDataStoragePathPos);

      // Iterate over all the contents of the folder and read all the images
      // it contains.
      for (std::vector<boost::filesystem::path>::const_iterator it
          (pathVector.begin());
          it != pathVector.end(); ++it)
      {
        cv::Mat img = cv::imread(it->c_str());
        if (!img.data)
        {
          ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Could not read "
              "image %s,proceeding to next pattern!", it->c_str());
          continue;
        }
        // Add the current view to the collection.
        imageCollection.push_back(img);
        imageNames.push_back(it->filename().stem().string());
      }
      if ( imageCollection.size() <= 0 || imageNames.size() <= 0)
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : No images were read!");
        return false;
      }

      // Finished parsing the images.
      // We will now calculate the descriptors of the pattern
      // from all the generating views!
      std::vector<cv::Point2f> boundingBox;
      std::vector<cv::KeyPoint> multiViewKeypoints;
      cv::Mat multiViewDescriptors;
      return multiViewTraining(dirPath,
          imageCollection, imageNames, homographies);
    }

    /**
     * @brief This method calculates and saves the features for a list of
     * images in the provided path.
     * @param dirPath[const boost::filesystem::path&]: The path to the images.
     * @param images[const std::vector<cv::Mat>]&: The list of images.
     * @param imageNames[const std::vector<std::string>&]: The list of image
     * names.
     * @param homographies[const std::map<std::string, cv::Mat>&]:
     * The container that contains for each view the corresponding homography.
     * @return bool : Flags that indicates success.
     */
    bool PlanarPatternTrainer::multiViewTraining(
        const boost::filesystem::path& dirPath,
        const std::vector<cv::Mat>& images,
        const std::vector<std::string>& imageNames,
        const std::map<std::string, cv::Mat>& homographies)
    {
      // The vector that contains all the keypoints found in the training set.
      std::vector<cv::Point2f> multiViewKeypoints;
      // The array of all the descriptors detected in the training set.
      cv::Mat multiViewDescriptors;

      cv::Mat frontViewDescriptors;
      std::vector<cv::KeyPoint> frontViewKeyPoints;
      std::vector<cv::Point2f> boundingBox;
      // Calculate the features for the frontal view of the pattern.
      this->getFeatures(images[0], &frontViewDescriptors,
          &frontViewKeyPoints, &boundingBox);

      multiViewDescriptors = frontViewDescriptors;
      std::vector<cv::Mat> synthViewDescriptors;
      std::vector<std::vector<cv::KeyPoint> > synthViewKeyPoints;

      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Starting to calculate"
          " features! ");
      // Find the features for all the images except the front view.
      this->getFeatures(std::vector<cv::Mat>(images.begin() + 1, images.end()),
          &synthViewDescriptors, &synthViewKeyPoints);

      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Finished calculating"
          " features! ");

      // Insert the frontal view keypoints.
      for (int i = 0 ; i < frontViewKeyPoints.size(); i++)
      {
        multiViewKeypoints.push_back(frontViewKeyPoints[i].pt);
      }

      // Perform the inverse transformations on the keypoints and store the
      // result.

      cv::Mat homography;
      std::map<std::string, cv::Mat>::const_iterator it;
      std::vector<cv::Point2f> viewKeyPoints;

      // For every image multiViewKeypoints.
      for (int i = 0 ; i < synthViewKeyPoints.size() ; i++)
      {
        // Get the homography that corresponds to the current view.
        it= homographies.find(imageNames[i + 1]);
        if (it == homographies.end())
        {
          ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Image %s has no"
              " homography stored!",
              imageNames[i + 1].c_str());
          continue;
        }
        homography = it->second;

        // Copy the keypoints detected in the i-th view.
        for (int j = 0 ; j < synthViewKeyPoints[i].size(); j++)
        {
          viewKeyPoints.push_back(synthViewKeyPoints[i][j].pt);
        }
        // Apply the inverse homography transform to get points in the original
        // coordinate frame.
        cv::perspectiveTransform(viewKeyPoints, viewKeyPoints,
            homography.inv());

        // Add the transformed points to the vector containing all the
        // keypoints.
        multiViewKeypoints.insert(multiViewKeypoints.end(),
            viewKeyPoints.begin(), viewKeyPoints.end());
        viewKeyPoints.clear();
        // Concatenate the descriptors matrices.
        cv::vconcat(synthViewDescriptors[i], multiViewDescriptors,
            multiViewDescriptors);
      }

      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Performing kmeans "
          "clustering on the set of descriptors!");
      cv::Mat labels;
      cv::Mat centroids;
      cv::kmeans(multiViewDescriptors, frontViewKeyPoints.size(), labels,
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
            10, 0.1), 3, cv::KMEANS_PP_CENTERS, centroids);

      ROS_INFO("Finished kmeans clustering!");
      cv::BFMatcher matcher(cv::NORM_L2);

      std::vector<std::vector<cv::DMatch> > matches;

      matcher.knnMatch(frontViewDescriptors, centroids, matches, 2);

      std::vector<cv::Point2f> bestKeypoints;

      for (int i = 0; i <  matches.size() ; i++)
      {
        // if(matches[i][0].distance < ratio*matches[i][1].distance)
        bestKeypoints.push_back(
            frontViewKeyPoints[matches[i][0].queryIdx].pt);
        std::cout << matches[i][0].distance << std::endl;
      }

      // saveDataToFile(dirPath, multiViewDescriptors, multiViewKeypoints,
      // boundingBox);
      // saveDataToFile(dirPath, centroids, multiViewKeypoints,
      // boundingBox);
      bool saveFlag = saveDataToFile(dirPath, centroids, bestKeypoints,
          boundingBox);

      return saveFlag;
    }

    /**
     * @brief This method reads the image whose path is dirPath,
     * calculates features and keypoints and stores them in a
     * corresponding xml file.
     * @param dirPath[const boost::filesystem::path&]: The path to the current
     * pattern.
     * @return bool : Flags that indicates success.
     */
    bool PlanarPatternTrainer::singleViewTraining(const boost::filesystem::path&
        dirPath)
    {
      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Reading image : %s ",
          dirPath.filename().c_str());
      cv::Mat descriptors;
      std::vector<cv::KeyPoint> keyPoints;
      std::vector<cv::Point2f> boundingBox;

      cv::Mat img = cv::imread(dirPath.c_str());
      if (!img.data)
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Could not read image,"
            "proceeding to next pattern!");
        return false;
      }
      // Calculate the image features.
      this->getFeatures(img, &descriptors, &keyPoints, &boundingBox);
      // Save the training data for the i-th image.

      this->saveDataToFile(dirPath, descriptors, keyPoints,
          boundingBox);
      return true;
    }

    /**
     * @brief Saves the training data to a proper XML file.
     * @param patternName [const std::string &] : The name of the pattern.
     * @param descriptors [const cv::Mat &] : The descriptors of the pattern.
     * @param keyPoints [const std::vector<cv::Point2f>] : The key points
     * detected on the pattern.
     * @return bool : Flags that indicates success.
     */
    bool PlanarPatternTrainer::saveDataToFile(
        const boost::filesystem::path &patternPath,
        const cv::Mat &descriptors ,
        const std::vector<cv::Point2f> &keyPoints,
        const std::vector<cv::Point2f> &boundingBox)
    {
      // Create the file name where the results will be stored.
      std::string fileName(patternPath.filename().string());

      // Remove the file extension.
      fileName = fileName.substr(0, fileName.find("."));


      // Properly choose the name of the data file.
      // TO DO : Change hard coded strings to yaml params.
      boost::filesystem::path resultPath(packagePath_ + "/data" + "/training"
          + "/" + this->getFeatureType() + "/" + fileName);

      // fileName = path + fileName;
      // ROS_INFO("DEBUG MESSAGE : Saving fileName %s", fileName.c_str());
      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Saving fileName %s",
          resultPath.c_str());

      // Opening the xml file for writing .
      cv::FileStorage fs(resultPath.string() + ".xml",
          cv::FileStorage::WRITE);

      if (!fs.isOpened())
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Could not open the"
            " file name: %s", resultPath.c_str());
        return false;
      }

      // Enter the name of the pattern.
      fs << "PatternName" << fileName;

      // Save the descriptors.
      fs << "Descriptors" << descriptors;

      // Calculate the number of keypoints found.
      int keyPointsNum = keyPoints.size();
      // Store the detected keypoints.
      fs << "PatternKeypoints" << "[";
      for (int i = 0 ; i < keyPointsNum; i++)
        fs << "{" << "Keypoint" <<  keyPoints[i] << "}";
      fs << "]";

      // Store the bounding box for the pattern.
      fs << "BoundingBox" << "[";
      for (int i = 0 ; i < boundingBox.size(); i++)
        fs << "{" << "Corner" <<  boundingBox[i] << "}";

      fs << "]";

      // Close the xml file .
      fs.release();

      return true;
    }

    /**
     * @brief Saves the training data to a proper XML file.
     * @param patternName [const std::string &] : The name of the pattern.
     * @param descriptors [const cv::Mat &] : The descriptors of the pattern.
     * @param keyPoints [const std::vector<cv::Keypoint>] : The key points
     * detected on the pattern.
     * @param boundingBox[const std::vector<cv::Point2f>&] : The vector
     * containing the bounding box of the pattern.
     * @return bool : Flag that indicates success.
     */
    bool PlanarPatternTrainer::saveDataToFile(
        const boost::filesystem::path& patternPath,
        const cv::Mat& descriptors ,
        const std::vector<cv::KeyPoint>& keyPoints,
        const std::vector<cv::Point2f>& boundingBox)
    {
      // Create the file name where the results will be stored.
      std::string fileName(patternPath.filename().string());

      // Remove the file extension.
      fileName = fileName.substr(0, fileName.find("."));


      // Properly choose the name of the data file.
      // TO DO : Change hard coded strings to yaml params.
      boost::filesystem::path resultPath(packagePath_ + "/data"
          + "/training" + "/" + this->getFeatureType() + "/" + fileName);

      // fileName = path + fileName;
      // ROS_INFO("DEBUG MESSAGE : Saving fileName %s", fileName.c_str());
      ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Saving fileName"
          " %s", resultPath.c_str());

      // Opening the xml file for writing .
      cv::FileStorage fs(resultPath.string() + ".xml",
          cv::FileStorage::WRITE);

      if (!fs.isOpened())
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Could not open the file"
            "name: %s", resultPath.c_str());
        return false;
      }

      // Enter the name of the pattern.
      fs << "PatternName" << fileName;

      // Save the descriptors.
      fs << "Descriptors" << descriptors;

      // Calculate the number of keypoints found.
      int keyPointsNum = keyPoints.size();
      // Store the detected keypoints.
      fs << "PatternKeypoints" << "[";
      for (int i = 0 ; i < keyPointsNum ; i++ )
        fs << "{" << "Keypoint" <<  keyPoints[i].pt << "}";
      fs << "]";

      // Store the bounding box for the pattern.
      fs << "BoundingBox" << "[";
      for (int i = 0 ; i < boundingBox.size() ; i++ )
        fs << "{" << "Corner" <<  boundingBox[i] << "}";

      fs << "]";

      // Close the xml file .
      fs.release();

      return true;
    }

    /**
     * @brief Iterate over the file containing the homographies and store them
     * in a vector.
     * @param homographyFilePath[const boost::filesystem::path&]: The path to
     * the file containing the homographies used to create each synthetic
     * view. Each line must have the name of the image it corresponds and
     * the matrix in  row wise form as csv values.
     * @param homographyMatrices[std::map<cv::Mat>*]: A mapping from the image
     * names to the corresponding homography matrices.
     */
    bool PlanarPatternTrainer::getHomographyMatrices(
        const boost::filesystem::path& homographyFilePath,
        std::map<std::string, cv::Mat>* homographyMatrices)
    {
      // A container for each line.
      std::string line;
      // Open the file.
      std::fstream file(homographyFilePath.c_str(), std::fstream::in);
      // The vector for the tokens.
      std::vector<std::string> tokens;
      // The vector used to parse to separate the float values.
      std::vector<std::string> coeffs;
      // The vector containing the actual numerical values of the coefficients
      // of the homography matrix.
      std::vector<float> matrix;
      // The OpenCV matrix that will be stored.
      cv::Mat homography(3, 3, CV_32FC1);

      std::string imageName;

      if (!file)
      {
        ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Could not find %s"
            " in path : %s!",
            homographyFilePath.filename().c_str(),
            homographyFilePath.parent_path().c_str());
        return false;
      }
      // Iterate over each line of the file.
      while (std::getline(file, line))
      {
        if (line.empty())
          continue;
        // Split the line to get the name of the file.
        boost::split(tokens, line, boost::is_any_of(", :"));
        for (int i = 0 ; i < tokens.size(); i++ )
        {
          if ( (tokens[i].find("view") != std::string::npos))
          {
            imageName = tokens[i];
          }
          else if ( tokens[i].compare(std::string()) == 0)
          {
            continue;
          }
          else
          {
            coeffs.push_back(tokens[i]);
          }
        }
        if (coeffs.size() <= 0 )
        {
          ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : No homography matrix"
              " coefficients were read!");
          return false;
        }
        std::stringstream ss(std::stringstream::in | std::stringstream::out);
        float temp;
        for (int i = 0 ; i < coeffs.size(); i++)
        {
          ss << coeffs[i];
          ss >> temp;
          matrix.push_back(temp);
          ss.clear();
        }
        if (matrix.size() > 9 )
        {
          ROS_ERROR("[PANDORA_VISION_HAZMAT_TRAINER] : Invalid number of"
              " entries for the homography file!");
          return false;
        }

        homography = cv::Mat(matrix, true);
        homography = homography.reshape(0, 3);
        // Insert the current value in the map and check if it is present.
        if ( !homographyMatrices->insert(std::pair<std::string, cv::Mat>
              (imageName, homography)).second)
        {
          ROS_INFO("[PANDORA_VISION_HAZMAT_TRAINER] : Value for key : %s"
              " is already present and will not be "
              "changed!\n Check your dataset!", imageName.c_str());
        }

        matrix.clear();
        coeffs.clear();
        tokens.clear();
      }
      file.close();

      // Check if any homography was read.
      return homographyMatrices->size() > 0;
    }
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision

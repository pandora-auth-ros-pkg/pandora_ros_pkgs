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
 * Authors:
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

#include "pandora_vision_victim/utilities/file_utilities.h"
#include "pandora_vision_victim/feature_extractors/rgb_feature_extraction.h"
#include "pandora_vision_victim/feature_extractors/depth_feature_extraction.h"
#include "pandora_vision_victim/classifiers/abstract_classifier.h"
#include <ros/ros.h>


namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @brief Constructor. Initialize member variables.
   */
  AbstractClassifier::AbstractClassifier(const ros::NodeHandle& nh,
      const std::string& datasetPath, const std::string& classifierType,
      const std::string& imageType)
  {
    datasetPath_ = datasetPath;
    imageType_ = imageType;
    classifierType_ = classifierType;

    nodeMessagePrefix_ = "[PANDORA_VISION_VICTIM_" + boost::to_upper_copy<std::string>(imageType)
        + "_" + boost::to_upper_copy<std::string>(classifierType) + "]";

    packagePath_ = ros::package::getPath("pandora_vision_victim");

    featureExtractionUtilities_.reset(new FeatureExtractionUtilities());

    filesDirectory_ = packagePath_ + "/data/";

    const std::string filePrefix = filesDirectory_ + imageType_ + "_";
    trainingFeaturesMatrixFile_ = filePrefix + "training_features_matrix.xml";
    testFeaturesMatrixFile_ = filePrefix + "test_features_matrix.xml";
    trainingLabelsMatrixFile_ = filePrefix + "training_labels_matrix.xml";
    testLabelsMatrixFile_ = filePrefix + "test_labels_matrix.xml";
    resultsFile_ = filePrefix + classifierType_ + "_results.xml";
    classifierFile_ = filePrefix + classifierType_ +  "_classifier.xml";

    const std::string trainingDatasetPath = datasetPath_;  // + "/Training_Images";
    // const std::string trainingDatasetPath = datasetPath_ + "/Training_Images";
    std::cout << trainingDatasetPath << std::endl;
    boost::filesystem::path trainingDirectory(trainingDatasetPath);
    trainingDirectory_ = trainingDirectory;

    trainingAnnotationsFile_ = filePrefix + "training_annotations.txt";
    std::cout << trainingAnnotationsFile_ << std::endl;
    int numTrainingFiles = file_utilities::findNumberOfAnnotations(trainingAnnotationsFile_);

    const std::string testDatasetPath = datasetPath_;  // + "/Test_Images";
    // const std::string testDatasetPath = datasetPath_ + "/Test_Images";
    std::cout << testDatasetPath << std::endl;
    boost::filesystem::path testDirectory(testDatasetPath);
    testDirectory_ = testDirectory;

    testAnnotationsFile_ = filePrefix + "test_annotations.txt";
    int numTestFiles = file_utilities::findNumberOfAnnotations(testAnnotationsFile_);

    std::string paramFile = packagePath_ + "/config/" + imageType + "_victim_training_params.yaml";
    cv::FileStorage fs(paramFile, cv::FileStorage::READ);
    fs.open(paramFile, cv::FileStorage::READ);

    std::cout << paramFile << std::endl;
    if (!fs.isOpened())
    {
      ROS_ERROR("Could not open : %s !", paramFile.c_str());
      ROS_ERROR("The training node will now shut down!");
      ROS_BREAK();
    }

    cv::FileNode classifierNode = fs[classifierType_];
    std::string trainingSetExtraction = classifierNode["training_set_feature_extraction"];
    std::string testSetExtraction = classifierNode["test_set_feature_extraction"];
    std::string loadModel = classifierNode["load_classifier_model"];
    std::string pcaAnalysis = classifierNode["do_pca_analysis"];

    trainingSetFeatureExtraction_ = trainingSetExtraction.compare("true") == 0;
    testSetFeatureExtraction_ = testSetExtraction.compare("true") == 0;
    loadClassifierModel_ = loadModel.compare("true") == 0;
    doPcaAnalysis_ = pcaAnalysis.compare("true") == 0;
    typeOfNormalization_ = static_cast<int>(classifierNode["type_of_normalization"]);

    fs.release();

    if (imageType_.compare("rgb") == 0)
    {
      featureExtraction_[imageType_].reset(new RgbFeatureExtraction(classifierType_));

      numFeatures_ = featureExtraction_[imageType_]->getFeatureNumber();
    }
    else if (imageType_.compare("depth") == 0)
    {
      featureExtraction_[imageType_].reset(new DepthFeatureExtraction(classifierType_));

      numFeatures_ = featureExtraction_[imageType_]->getFeatureNumber();
    }
    else if (imageType_.compare("rgbd") == 0)
    {
      featureExtraction_["rgb"].reset(new RgbFeatureExtraction(classifierType_));
      featureExtraction_["depth"].reset(new DepthFeatureExtraction(classifierType_));
    }
    else
    {
      ROS_ERROR("Invalid image type provided!");
      ROS_ERROR("The system will now shut down!");
      ROS_BREAK();
    }


    ROS_INFO("Created Abstract Classifier Instance!");
  }

  /**
   * @brief Destructor
   */
  AbstractClassifier::~AbstractClassifier()
  {
    ROS_DEBUG("[victim_node] : Destroying Abstract Classifier instance");
  }

  /**
   * @brief This function normalizes the features and saves normalization
   * parameters in a file.
   * @param featuresMatrix [cv::Mat*] The features matrix to be normalized.
   */
  void AbstractClassifier::normalizeFeaturesAndSaveNormalizationParameters(
      cv::Mat* featuresMatrix)
  {
    if (typeOfNormalization_ == 0)
    {
      return;
    }

    std::string normalizationParamOne;
    std::string normalizationParamTwo;

    std::string normalizationParamOneTag;
    std::string normalizationParamTwoTag;

    std::vector<double> normalizationParamOneVector;
    std::vector<double> normalizationParamTwoVector;

    if (typeOfNormalization_ == 1)
    {
      double newMin = -1.0;
      double newMax = 1.0;
      featureExtractionUtilities_->findMinMaxParameters(newMax, newMin,
          featuresMatrix, &normalizationParamOneVector,
          &normalizationParamTwoVector);

      normalizationParamOneTag = "min";
      normalizationParamTwoTag = "max";

      normalizationParamOne = imageType_ + "_" + classifierType_ + "_min_values.xml";
      normalizationParamTwo = imageType_ + "_" + classifierType_ + "_max_values.xml";
    }
    else
    {
      featureExtractionUtilities_->findZScoreParameters(featuresMatrix,
          &normalizationParamOneVector, &normalizationParamTwoVector);

      normalizationParamOneTag = "mean";
      normalizationParamTwoTag = "std_dev";

      normalizationParamOne = imageType_ + "_" + classifierType_ + "_mean_values.xml";
      normalizationParamTwo = imageType_ + "_" + classifierType_ + "_standard_deviation_values.xml";
    }

    std::string normalizationParamOnePath = filesDirectory_ + normalizationParamOne;
    std::string normalizationParamTwoPath = filesDirectory_ + normalizationParamTwo;

    file_utilities::saveToFile(normalizationParamOnePath,
        normalizationParamOneTag,
        cv::Mat(normalizationParamOneVector));
    file_utilities::saveToFile(normalizationParamTwoPath,
        normalizationParamTwoTag,
        cv::Mat(normalizationParamTwoVector));
  }

  /**
   * @brief This function loads normalization parameters and normalizes the
   * input features matrix.
   * @param featuresMatrix [cv::Mat*] The features matrix to be normalized.
   */
  void AbstractClassifier::loadNormalizationParametersAndNormalizeFeatures(
      cv::Mat* featuresMatrix)
  {
    if (typeOfNormalization_ == 0)
    {
      return;
    }

    std::string normalizationParamOne;
    std::string normalizationParamOnePath;
    std::string normalizationParamTwo;
    std::string normalizationParamTwoPath;

    std::string normalizationParamOneTag;
    std::string normalizationParamTwoTag;

    std::vector<double> normalizationParamOneVector;
    std::vector<double> normalizationParamTwoVector;

    if (typeOfNormalization_ == 1)
    {
      double newMax = 1.0;
      double newMin = -1.0;

      normalizationParamOne = imageType_ + "_" + classifierType_ + "_min_values.xml";
      normalizationParamOnePath = filesDirectory_ + normalizationParamOne;
      normalizationParamTwo = imageType_ + "_" + classifierType_ + "_max_values.xml";
      normalizationParamTwoPath = filesDirectory_ + normalizationParamTwo;

      normalizationParamOneTag = "min";
      normalizationParamTwoTag = "max";
      normalizationParamOneVector = file_utilities::loadFiles(
          normalizationParamOnePath,
          normalizationParamOneTag);
      normalizationParamTwoVector = file_utilities::loadFiles(
          normalizationParamTwoPath,
          normalizationParamTwoTag);
      featureExtractionUtilities_->performMinMaxNormalization(newMax, newMin,
          featuresMatrix, normalizationParamOneVector,
          normalizationParamTwoVector);
    }
    else
    {
      normalizationParamOne = imageType_ + "_" + classifierType_ + "_mean_values.xml";
      normalizationParamOnePath = filesDirectory_ + normalizationParamOne;
      normalizationParamTwo = imageType_ + "_" + classifierType_ + "_standard_deviation_values.xml";
      normalizationParamTwoPath = filesDirectory_ + normalizationParamTwo;

      normalizationParamOneTag = "mean";
      normalizationParamTwoTag = "std_dev";
      normalizationParamOneVector = file_utilities::loadFiles(
          normalizationParamOnePath,
          normalizationParamOneTag);
      normalizationParamTwoVector = file_utilities::loadFiles(
          normalizationParamTwoPath,
          normalizationParamTwoTag);
      if (featuresMatrix->empty() || normalizationParamOneVector.empty() ||
          normalizationParamTwoVector.empty())
      {
        std::cout << "Cannot perform Z Score Normalization for test features, invalid input matrices!"
          << std::endl;
        exit(-1);
      }
      featureExtractionUtilities_->performZScoreNormalization(
          featuresMatrix, normalizationParamOneVector,
          normalizationParamTwoVector);
    }
  }

  /**
   * @brief
   */
  bool AbstractClassifier::constructBagOfWordsVocabulary(
      const boost::filesystem::path& directory,
      const std::string& annotationsFile)
  {
    return featureExtraction_[imageType_]->constructBagOfWordsVocabulary(directory,
        annotationsFile);
  }

  /**
   * @brief This function constructs the features matrix, i.e. the feature
   * vectors of a set of images.
   * @param directory [const boost::filesystem::path&] The directory that
   * contains the set of images for the feature extraction.
   * @param annotationsFile [const std::string&] The name of the file that
   * contains the class attributes of the images to be processed.
   * @param featuresMat [cv::Mat*] The features matrix.
   * @param labelsMat [cv::Mat*] The matrix that contains the class attributes
   * for the processed set of images.
   * @return void
   */
  void AbstractClassifier::constructFeaturesMatrix(
      const boost::filesystem::path& directory,
      const std::string& annotationsFile,
      cv::Mat* featuresMat, cv::Mat* labelsMat)
  {
    featureExtraction_[imageType_]->constructFeaturesMatrix(directory,
        annotationsFile, featuresMat, labelsMat);
  }

  /**
   * @brief This function evaluates the classifier model, based on the predicted
   * and the actual class attributes.
   * @param predicted [const cv::Mat&] The predicted class attributes.
   * @param actual [const cv::Mat&] The actual class attributes.
   * @return void
   */
  void AbstractClassifier::evaluate(const cv::Mat& predicted, const cv::Mat& actual)
  {
    assert(predicted.rows == actual.rows);
    int truePositives = 0;
    int falsePositives = 0;
    int trueNegatives = 0;
    int falseNegatives = 0;
    for (int ii = 0; ii < actual.rows; ii++)
    {
      float p = predicted.at<float>(ii, 0);
      float a = actual.at<float>(ii, 0);

      if (p >= 0.0 && a >= 0.0)
        truePositives++;
      else if (p <= 0.0 && a <= 0.0)
        trueNegatives++;
      else if (p >= 0.0 && a <= 0.0)
        falsePositives++;
      else if (p <= 0.0 && a >= 0.0)
        falseNegatives++;
    }
    accuracy_ = static_cast<float>(truePositives + trueNegatives) /
              (truePositives + trueNegatives + falsePositives + falseNegatives);
    precision_ = static_cast<float>(truePositives) /
              (truePositives + falsePositives);
    recall_ = static_cast<float>(truePositives) /
              (truePositives + falseNegatives);
    fmeasure_ = (2.0 * truePositives) /
              (2.0 * truePositives + falseNegatives + falsePositives);

    std::cout << "True Positives = " << truePositives << std::endl;
    std::cout << "True Negatives = " << trueNegatives << std::endl;
    std::cout << "False Positives = " << falsePositives << std::endl;
    std::cout << "False Negatives = " << falseNegatives << std::endl;

    std::cout << classifierType_ << " Accuracy = " << accuracy_ << std::endl;
    std::cout << classifierType_ << " Precision = " << precision_ << std::endl;
    std::cout << classifierType_ << " Recall = " << recall_ << std::endl;
    std::cout << classifierType_ << " F-Measure = " << fmeasure_ << std::endl;
  }

  /**
   * @brief Function that implements the training for the subsystems
   * according to the given training sets. It applies a classification
   * algorithm and extracts a suitable model.
   * @return void
   */
  void AbstractClassifier::trainAndValidate(void)
  {
    ROS_INFO("Starting the Training Procedure for the %s Classifier!",
        boost::to_upper_copy<std::string>(classifierType_).c_str());

    int numTrainingFiles = file_utilities::findNumberOfAnnotations(trainingAnnotationsFile_);
    int numTestFiles = file_utilities::findNumberOfAnnotations(testAnnotationsFile_);

    cv::Mat trainingFeaturesMat = cv::Mat::zeros(numTrainingFiles, numFeatures_, CV_64FC1);
    cv::Mat trainingLabelsMat = cv::Mat::zeros(numTrainingFiles, 1, CV_64FC1);
    cv::Mat testFeaturesMat = cv::Mat::zeros(numTestFiles, numFeatures_, CV_64FC1);
    cv::Mat testLabelsMat = cv::Mat::zeros(numTestFiles, 1, CV_64FC1);

    if (loadClassifierModel_ && file_utilities::exist(classifierFile_.c_str()))
    {
      ROS_INFO("Loading model for %s Classifier", boost::to_upper_copy<std::string>(classifierType_).c_str());
      this->load(classifierFile_);
    }
    else
    {
      if (file_utilities::exist(trainingFeaturesMatrixFile_.c_str()) == false ||
          trainingSetFeatureExtraction_)
      {
        std::cout << "Create necessary training matrix" << std::endl;
        std::string prefix = "training_";

        bool vocabularyNeeded = constructBagOfWordsVocabulary(trainingDirectory_,
            trainingAnnotationsFile_);

        constructFeaturesMatrix(trainingDirectory_, trainingAnnotationsFile_,
            &trainingFeaturesMat, &trainingLabelsMat);

        std::cout << "Normalize features" << std::endl;
        normalizeFeaturesAndSaveNormalizationParameters(&trainingFeaturesMat);

        trainingFeaturesMat.convertTo(trainingFeaturesMat, CV_32FC1);
        trainingLabelsMat.convertTo(trainingLabelsMat, CV_32FC1);

        file_utilities::saveFeaturesInFile(trainingFeaturesMat, trainingLabelsMat,
            prefix, trainingFeaturesMatrixFile_, trainingLabelsMatrixFile_,
            imageType_);

        if (vocabularyNeeded)
        {
          std::cout << "Save bag of words vocabulary" << std::endl;
          const std::string bagOfWordsFile = imageType_ + "_" + classifierType_ + "_bag_of_words.xml";
          const std::string bagOfWordsFilePath = filesDirectory_ + bagOfWordsFile;
          file_utilities::saveToFile(bagOfWordsFilePath, "bag_of_words",
              featureExtraction_[imageType_]->getBagOfWordsVocabulary());
        }
      }
      else
      {
        trainingFeaturesMat = file_utilities::loadFiles(
            trainingFeaturesMatrixFile_, "training_features_mat");
        trainingLabelsMat = file_utilities::loadFiles(
            trainingLabelsMatrixFile_, "training_labels_mat");
      }

      // Start Training Process
      std::cout << "Starting training process for the " << boost::to_upper_copy<std::string>(imageType_)
        << " images" << std::endl;

      this->train(trainingFeaturesMat, trainingLabelsMat, classifierFile_);

      std::cout << "Finished training process" << std::endl;
    }
    if (file_utilities::exist(testFeaturesMatrixFile_.c_str()) == false ||
        testSetFeatureExtraction_)
    {
      std::cout << "Create necessary test matrix" << std::endl;
      std::string prefix = "test_";

      constructFeaturesMatrix(testDirectory_, testAnnotationsFile_,
          &testFeaturesMat, &testLabelsMat);

      loadNormalizationParametersAndNormalizeFeatures(&testFeaturesMat);

      testFeaturesMat.convertTo(testFeaturesMat, CV_32FC1);
      testLabelsMat.convertTo(testLabelsMat, CV_32FC1);

      file_utilities::saveFeaturesInFile(testFeaturesMat, testLabelsMat,
          prefix, testFeaturesMatrixFile_, testLabelsMatrixFile_, imageType_);
    }
    else
    {
      testFeaturesMat = file_utilities::loadFiles(
          testFeaturesMatrixFile_, "test_features_mat");
      testLabelsMat = file_utilities::loadFiles(
          testLabelsMatrixFile_, "test_labels_mat");
    }

    cv::Mat results = cv::Mat::zeros(numTestFiles, 1, CV_32FC1);

    validate(testFeaturesMat, &results);

    file_utilities::saveToFile(resultsFile_, "results", results);
    evaluate(results, testLabelsMat);
  }

}  // namespace pandora_vision_victim
}  // namespace pandora_vision


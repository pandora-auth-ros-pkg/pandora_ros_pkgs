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
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/
#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>

#include "pandora_vision_victim/classifiers/rgbd_svm_classifier.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @brief Constructor
   */
  RgbdSvmClassifier::RgbdSvmClassifier(const  ros::NodeHandle& nh,
      const std::string& datasetPath,
      const std::string& classifierType,
      const std::string& imageType)
      : AbstractClassifier(nh, datasetPath, classifierType, imageType)
  {
    ROS_INFO("[victim_node] : Creating rgbd SVM training instance!");

    int svmType;
    if (!nh.getParam(imageType_ + "/svm_type", svmType))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the type of the SVM Classifier!");
      svmType = CvSVM::C_SVC;
      ROS_INFO_STREAM(nodeMessagePrefix_ << ": Using the default type " << svmType);
    }

    int kernelType;
    if (!nh.getParam(imageType_ + "/kernel_type", kernelType))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the kernel type of the SVM Classifier!");
      kernelType = CvSVM::RBF;
      ROS_INFO_STREAM(nodeMessagePrefix_ << ": Using the default type " << kernelType);
    }

    double svmDegree = 0.0;
    if (kernelType == CvSVM::POLY)
    {
      if (!nh.getParam(imageType_ + "/degree", svmDegree))
      {
        ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
            << "the degree of the SVM Classifier!");
        ROS_BREAK();
      }
    }

    double svmGamma = 1.0;
    if (kernelType == CvSVM::POLY || kernelType == CvSVM::RBF || kernelType == CvSVM::SIGMOID)
    {
      if (!nh.getParam(imageType_ + "/gamma", svmGamma))
      {
        ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
            << "the gamma parameter for the SVM Classifier!");
        ROS_BREAK();
      }
    }

    double svmCoef0 = 0.0;
    if (kernelType == CvSVM::POLY || kernelType == CvSVM::SIGMOID)
    {
      if (!nh.getParam(imageType_ + "/coef0", svmCoef0))
      {
        ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
            << "the coef0 parameter for the SVM Classifier!");
        ROS_BREAK();
      }
    }

    double svmC = 1.0;
    if (svmType == CvSVM::C_SVC || svmType == CvSVM::EPS_SVR || svmType == CvSVM::NU_SVR)
    {
      if (!nh.getParam(imageType_ + "/C", svmC))
      {
        ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
            << "the C parameter for the SVM classifier!");
        ROS_BREAK();
      }
    }

    double svmNu = 0.0;
    if (svmType == CvSVM::NU_SVC || svmType == CvSVM::ONE_CLASS || svmType == CvSVM::NU_SVR)
    {
      if (!nh.getParam(imageType_ + "/nu", svmNu))
      {
        ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
            << "the Nu parameter for the SVM classifier!");
        ROS_BREAK();
      }
    }
    double svmP = 0.0;
    if (svmType == CvSVM::EPS_SVR)
    {
      if (!nh.getParam(imageType_ + "/p", svmP))
      {
        ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
            << "the p parameter for the SVM classifier!");
        ROS_BREAK();
      }
    }
    svmParams_.svm_type = svmType;
    svmParams_.kernel_type = kernelType;

    svmParams_.degree = svmDegree;  //!< POLY
    svmParams_.gamma = svmGamma;  //!< POLY/RBF/SIGMOID
    svmParams_.coef0 = svmCoef0;  //!< POLY/SIGMOID
    svmParams_.C = svmC;  //!< CV_SVM_C_SVC, CV_SVM_EPS_SVR, CV_SVM_NU_SVR
    svmParams_.nu = svmNu;  //!< CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, CV_SVM_NU_SVR
    svmParams_.p = svmP;  //!< CV_SVM_EPS_SVR
    svmParams_.class_weights = NULL;  //!< CV_SVM_C_SVC
    svmParams_.term_crit = cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
        10000, 1e-6);

    // CvParamGrid CvParamGrid_C(pow(2.0,-5), pow(2.0,15), pow(2.0,2));
    // CvParamGrid CvParamGrid_gamma(pow(2.0,-20), pow(2.0,3), pow(2.0,2));
    // if (!CvParamGrid_C.check() || !CvParamGrid_gamma.check())
      // std::cout << "The grid is NOT VALID." << std::endl;

    bool autoTrain;
    if (!nh.getParam(imageType_ + "/auto_train", autoTrain))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the auto train parameter of the SVM Classifier!");
      autoTrain = true;
      ROS_INFO_STREAM(nodeMessagePrefix_ << ": Using the default value " << autoTrain);
    }
    autoTrain_ = autoTrain;

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
    std::string usePlattScaling = classifierNode["use_platt_scaling"];
    usePlattScaling_ = usePlattScaling.compare("true") == 0;

    fs.release();

    if (usePlattScaling_)
    {
      plattScalingPtr_.reset(new PlattScaling());
    }

    classifierPtr_.reset(new CvSVM());

    numRgbFeatures_ =  featureExtraction_["rgb"]->getFeatureNumber();
    numDepthFeatures_ =  featureExtraction_["depth"]->getFeatureNumber();
    numFeatures_ = numRgbFeatures_ + numDepthFeatures_;

    std::cout << numRgbFeatures_ << std::endl;
    std::cout << numDepthFeatures_ << std::endl;

    ROS_INFO("Created %s %s Training Instance.", imageType_.c_str(), classifierType_.c_str());
  }

  /**
   * @brief Destructor
   */
  RgbdSvmClassifier::~RgbdSvmClassifier()
  {
  }

  /**
   * @brief
   */
  std::vector<bool> RgbdSvmClassifier::constructBagOfWordsVocabulary(
      const boost::filesystem::path& rgbDirectory,
      const boost::filesystem::path& depthDirectory,
      const std::string& annotationsFile)
  {
    std::vector<bool> vocabulariesNeeded;
    vocabulariesNeeded.push_back(featureExtraction_["rgb"]->constructBagOfWordsVocabulary(rgbDirectory,
        annotationsFile));

    vocabulariesNeeded.push_back(featureExtraction_["depth"]->constructBagOfWordsVocabulary(depthDirectory,
        annotationsFile));
    return vocabulariesNeeded;
  }

  void RgbdSvmClassifier::constructFeaturesMatrix(
      const boost::filesystem::path& rgbDirectory,
      const boost::filesystem::path& depthDirectory,
      const std::string& annotationsFile,
      cv::Mat* featuresMat, cv::Mat* labelsMat)
  {
    cv::Mat rgbFeaturesMat = cv::Mat::zeros(featuresMat->rows, numRgbFeatures_, CV_64FC1);
    cv::Mat depthFeaturesMat = cv::Mat::zeros(featuresMat->rows, numDepthFeatures_, CV_64FC1);
    featureExtraction_["rgb"]->constructFeaturesMatrix(rgbDirectory,
        annotationsFile, &rgbFeaturesMat, labelsMat);
    featureExtraction_["depth"]->constructFeaturesMatrix(depthDirectory,
        annotationsFile, &depthFeaturesMat, labelsMat);
    int jj = 0;
    for (int ii = 0; ii < rgbFeaturesMat.cols; ii++)
    {
      rgbFeaturesMat.col(ii).copyTo(featuresMat->col(jj));
      jj += 1;
    }
    jj = rgbFeaturesMat.cols;
    for (int ii = 0; ii < depthFeaturesMat.cols; ii++)
    {
      depthFeaturesMat.col(ii).copyTo(featuresMat->col(jj));
      jj += 1;
    }
  }

  /**
   * @brief Trains the corresponding classifier using the input features and training labels.
   * @param trainingFeatures[const cv::Mat&] The matrix containing the features that describe the
   * training set
   * @param trainingLabels[const cv::Mat&] The corresponding labels that define the class of each
   * training sample.
   * @param classifierFileDest[const std::string&] The file where the classifier will be stored.
   * @return bool True on successfull completions, false otherwise.
   */
  bool RgbdSvmClassifier::train(const cv::Mat& trainingSetFeatures, const cv::Mat& trainingSetLabels,
      const std::string& classifierFileDest)
  {
    if (trainingSetFeatures.empty())
    {
      ROS_ERROR("[PANDORA_VISION_VICTIM_SVM]: The features matrix is empty!");
      return false;
    }
    if (trainingSetLabels.empty())
    {
      ROS_ERROR("[PANDORA_VISION_VICTIM_SVM]: The labels matrix is empty!");
      return false;
    }

    if (autoTrain_)
    {
      std::cout << "(SVM 'grid search' => may take some time!)" << std::endl;
      classifierPtr_->train_auto(trainingSetFeatures, trainingSetLabels, cv::Mat(), cv::Mat(),
          svmParams_, 10, CvSVM::get_default_grid(CvSVM::C),
          CvSVM::get_default_grid(CvSVM::GAMMA),
          CvSVM::get_default_grid(CvSVM::P),
          CvSVM::get_default_grid(CvSVM::NU),
          CvSVM::get_default_grid(CvSVM::COEF),
          CvSVM::get_default_grid(CvSVM::DEGREE),
          true);

      svmParams_ = classifierPtr_->get_params();
      std::cout << "Using optimal Parameters" << std::endl;
      std::cout << "degree=" << svmParams_.degree << std::endl;
      std::cout << "gamma=" << svmParams_.gamma << std::endl;
      std::cout << "coef0=" << svmParams_.coef0 << std::endl;
      std::cout << "C=" << svmParams_.C << std::endl;
      std::cout << "nu=" << svmParams_.nu << std::endl;
      std::cout << "p=" << svmParams_.p << std::endl;
    }
    else
    {
      classifierPtr_->train(trainingSetFeatures, trainingSetLabels, cv::Mat(), cv::Mat(), svmParams_);
    }

    if (usePlattScaling_)
    {
      cv::Mat predictionsMat = cv::Mat::zeros(trainingSetFeatures.rows, 1, CV_32FC1);
      for (int ii = 0; ii < trainingSetFeatures.rows; ii++)
      {
        float prediction = classifierPtr_->predict(trainingSetFeatures.row(ii), true);
        float classLabel = classifierPtr_->predict(trainingSetFeatures.row(ii), false);
        predictionsMat.at<double>(ii, 0) = static_cast<double>(prediction);
      }
      plattScalingPtr_->sigmoidTrain(predictionsMat, trainingSetLabels);
      std::string plattParametersFile = filesDirectory_ + imageType_ + "_" + classifierType_ + "_platt_scaling.xml";
      plattScalingPtr_->save(plattParametersFile);
    }

    classifierPtr_->save(classifierFile_.c_str());
    return true;
  }

  /**
   * @brief Validates the resulting classifier using the given features
   * extracted from the test set.
   * @param testSetFeatures[const cv::Mat&] The test set features matrix
   * @param validationResults[cv::Mat*] The results for the test set.
   * @return void
   */
  void RgbdSvmClassifier::validate(const cv::Mat& testSetFeatures, cv::Mat* validationResults)
  {
    // uncomment for ONE_CLASS SVM
    // for (int ii = 0; ii < results.rows; ii++)
    // for (int jj = 0; jj < results.cols; jj++)
    // if(results.at<float>(ii, jj) == 0)
    // results.at<float>(ii, jj) = -1;

    classifierPtr_->predict(testSetFeatures, *validationResults);
    return;
  }
  /**
   * @brief Function that implements the training for the subsystems
   * according to the given training sets. It applies SVM and extracts
   * a suitable model.
   * @return void
   */
  void RgbdSvmClassifier::trainAndValidate(void)
  {
    ROS_INFO("Starting the Training Procedure for the RGBD Classifier!");

    int numTrainingFiles = file_utilities::findNumberOfAnnotations(trainingAnnotationsFile_);
     std::cout << "numTrainingFiles=" << numTrainingFiles << std::endl;
     int numTestFiles = file_utilities::findNumberOfAnnotations(testAnnotationsFile_);

    cv::Mat trainingFeaturesMat = cv::Mat::zeros(numTrainingFiles, numFeatures_, CV_64FC1);
    std::cout << "numTestFiles=" << numTestFiles << std::endl;

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

        std::string rgbDatasetPath = trainingDirectory_.string();
        rgbDatasetPath += "/rgb";
        std::cout << rgbDatasetPath << std::endl;
        boost::filesystem::path rgbTrainingDirectory(rgbDatasetPath);

        std::string depthDatasetPath = trainingDirectory_.string();
        depthDatasetPath += "/depth";
        std::cout << depthDatasetPath << std::endl;
        boost::filesystem::path depthTrainingDirectory(depthDatasetPath);

        std::vector<bool> vocabulariesNeeded = constructBagOfWordsVocabulary(rgbTrainingDirectory,
            depthTrainingDirectory,
            trainingAnnotationsFile_);

        constructFeaturesMatrix(rgbTrainingDirectory, depthTrainingDirectory, trainingAnnotationsFile_,
            &trainingFeaturesMat, &trainingLabelsMat);

        std::cout << "Normalize features" << std::endl;
        normalizeFeaturesAndSaveNormalizationParameters(&trainingFeaturesMat);

        trainingFeaturesMat.convertTo(trainingFeaturesMat, CV_32FC1);
        trainingLabelsMat.convertTo(trainingLabelsMat, CV_32FC1);

        file_utilities::saveFeaturesInFile(trainingFeaturesMat, trainingLabelsMat,
            prefix, trainingFeaturesMatrixFile_, trainingLabelsMatrixFile_,
            imageType_);

        std::vector<std::string> imageTypesVec;
        imageTypesVec.push_back("rgb");
        imageTypesVec.push_back("depth");
        for (int ii = 0; ii < vocabulariesNeeded.size(); ii++)
        {
          if (vocabulariesNeeded[ii])
          {
            std::cout << "Save bag of words vocabulary" << std::endl;
            const std::string bagOfWordsFile = imageType_ + "_" + imageTypesVec[ii] +"_svm_bag_of_words.xml";
            const std::string bagOfWordsFilePath = filesDirectory_  + bagOfWordsFile;
            file_utilities::saveToFile(bagOfWordsFilePath, "bag_of_words",
                featureExtraction_[imageTypesVec[ii]]->getBagOfWordsVocabulary());
          }
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
      std::cout << "Starting training process for the " << boost::to_upper_copy<std::string>(imageType_);

      this->train(trainingFeaturesMat, trainingLabelsMat, classifierFile_);

      std::cout << "Finished training process" << std::endl;
    }
    if (file_utilities::exist(testFeaturesMatrixFile_.c_str()) == false ||
        testSetFeatureExtraction_)
    {
      std::cout << "Create necessary test matrix" << std::endl;
      std::string prefix = "test_";

      std::string rgbDatasetPath = testDirectory_.string();
      rgbDatasetPath += "/rgb";
      std::cout << rgbDatasetPath << std::endl;
      boost::filesystem::path rgbTestDirectory(rgbDatasetPath);

      std::string depthDatasetPath = testDirectory_.string();
      depthDatasetPath += "/depth";
      std::cout << depthDatasetPath << std::endl;
      boost::filesystem::path depthTestDirectory(depthDatasetPath);

      constructFeaturesMatrix(rgbTestDirectory, depthTestDirectory, testAnnotationsFile_,
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

    cv::Mat results = cv::Mat::zeros(numTestFiles, 1, CV_64FC1);
    validate(testFeaturesMat, &results);

    // std::cout << "results" << results.size() << std::endl << results <<std::endl <<std::endl;
    file_utilities::saveToFile(resultsFile_, "results", results);
    evaluate(results, testLabelsMat);
  }

}  // namespace pandora_vision_victim
}  // namespace pandora_vision


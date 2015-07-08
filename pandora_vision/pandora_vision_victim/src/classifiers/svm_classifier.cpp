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
*   Marios Protopapas
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include <string>

#include <ros/ros.h>

#include "pandora_vision_victim/classifiers/svm_classifier.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @brief Constructor
   */
  SvmClassifier::SvmClassifier(const ros::NodeHandle& nh,
      const std::string& datasetPath,
      const std::string& classifierType,
      const std::string& imageType)
      : AbstractClassifier(nh, datasetPath, classifierType, imageType)
  {
    ROS_INFO("[victim_node] : Creating SVM training instance!");

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

    if (doPcaAnalysis_)
    {
      pcaPtr_.reset(new PrincipalComponentAnalysis());
    }

    classifierPtr_.reset(new CvSVM());

    ROS_INFO("Created %s %s Training Instance.", imageType_.c_str(), classifierType_.c_str());
  }

  /**
   * @brief Destructor
   */
  SvmClassifier::~SvmClassifier()
  {
    ROS_DEBUG("[victim_node] : Destroying Svm training instance");
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
  bool SvmClassifier::train(const cv::Mat& trainingSetFeatures, const cv::Mat& trainingSetLabels,
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

    cv::Mat projectedTrainingSetFeatures;
    if (doPcaAnalysis_)
    {
      pcaPtr_->performPCA(trainingSetFeatures);
      pcaPtr_->project(trainingSetFeatures, &projectedTrainingSetFeatures);
      std::string pcaParametersFile = filesDirectory_ + imageType_ + "_" + classifierType_ + "_pca_parameters.xml";
      pcaPtr_->save(pcaParametersFile);
    }
    else
    {
      projectedTrainingSetFeatures = trainingSetFeatures.clone();
    }

    if (autoTrain_)
    {
      std::cout << "(SVM 'grid search' => may take some time!)" << std::endl;
      classifierPtr_->train_auto(projectedTrainingSetFeatures, trainingSetLabels, cv::Mat(), cv::Mat(),
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
      classifierPtr_->train(projectedTrainingSetFeatures, trainingSetLabels, cv::Mat(), cv::Mat(), svmParams_);
    }

    if (usePlattScaling_)
    {
      cv::Mat predictionsMat = cv::Mat::zeros(projectedTrainingSetFeatures.rows, 1, CV_64FC1);
      for (int ii = 0; ii < projectedTrainingSetFeatures.rows; ii++)
      {
        float prediction = classifierPtr_->predict(projectedTrainingSetFeatures.row(ii), true);
        float classLabel = classifierPtr_->predict(projectedTrainingSetFeatures.row(ii), false);
        predictionsMat.at<double>(ii, 0) = fabs(static_cast<double>(prediction)) * static_cast<double>(classLabel);
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
  void SvmClassifier::validate(const cv::Mat& testSetFeatures, cv::Mat* validationResults)
  {
    // uncomment for ONE_CLASS SVM
    // for (int ii = 0; ii < results.rows; ii++)
    // for (int jj = 0; jj < results.cols; jj++)
    // if(results.at<float>(ii, jj) == 0)
    // results.at<float>(ii, jj) = -1;

    cv::Mat projectedTestSetFeatures;
    if (doPcaAnalysis_)
    {
      pcaPtr_->project(testSetFeatures, &projectedTestSetFeatures);
    }
    else
    {
      projectedTestSetFeatures = testSetFeatures.clone();
    }

    classifierPtr_->predict(projectedTestSetFeatures, *validationResults);
    return;
  }

}  // namespace pandora_vision_victim
}  // namespace pandora_vision

/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors:
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include <string>
#include <vector>

#include "pandora_vision_victim/svm_classifier/depth_svm_training.h"

namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  DepthSvmTraining::DepthSvmTraining(const std::string& ns,
      int numFeatures, const std::string& datasetPath) :
      SvmTraining(ns, numFeatures, datasetPath)
  {
    std::string paramFile = package_path + "/config/depth_svm_training_params.yaml";
    cv::FileStorage fs(paramFile, cv::FileStorage::READ);
    fs.open(paramFile, cv::FileStorage::READ);

    std::string trainingSetExtraction = fs["training_set_feature_extraction"];
    std::string testSetExtraction = fs["test_set_feature_extraction"];
    std::string loadModel = fs["load_classifier_model"];
    std::string pcaAnalysis = fs["do_pca_analysis"];

    trainingSetFeatureExtraction_ = trainingSetExtraction.compare("false");
    testSetFeatureExtraction_ = testSetExtraction.compare("false");
    loadClassifierModel_ = loadModel.compare("false");
    doPcaAnalysis_ = pcaAnalysis.compare("false");
    typeOfNormalization_ = static_cast<int>(fs["type_of_normalization"]);

    fs.release();

    imageType_ = "depth_";
    featureExtraction_ = new DepthFeatureExtraction();
    std::cout << "Created Depth SVM Training Instance" << std::endl;
  }

  /**
   * @brief Destructor
   */
  DepthSvmTraining::~DepthSvmTraining()
  {
  }

  /**
   * @brief Function that implements the training for the subsystems
   * according to the given training sets. It applies SVM and extracts
   * a suitable model.
   * @return void
   */
  void DepthSvmTraining::trainSubSystem()
  {
    const std::string trainingFeaturesMatrixFile = imageType_ + "training_features_matrix.xml";
    const std::string testFeaturesMatrixFile = imageType_ + "test_features_matrix.xml";
    const std::string trainingLabelsMatrixFile = imageType_ + "training_labels_matrix.xml";
    const std::string testLabelsMatrixFile = imageType_ + "test_labels_matrix.xml";
    const std::string resultsFile = imageType_ + "results.xml";
    const std::string svmClassifierFile = imageType_ + "svm_classifier.xml";
    const std::string filesDirectory = package_path + "/data/";

    std::stringstream in_file_stream;
    std::stringstream in_test_file_stream;
    std::stringstream labels_mat_file_stream;
    std::stringstream test_labels_mat_file_stream;
    std::stringstream svm_file_stream;
    std::stringstream results_file_stream;

    in_file_stream << filesDirectory << trainingFeaturesMatrixFile;
    in_test_file_stream << filesDirectory << testFeaturesMatrixFile;
    labels_mat_file_stream << filesDirectory << trainingLabelsMatrixFile;
    test_labels_mat_file_stream << filesDirectory << testLabelsMatrixFile;
    svm_file_stream << filesDirectory << svmClassifierFile;
    results_file_stream << filesDirectory << resultsFile;

    const std::string trainingDatasetPath = path_to_samples + "/data/Training_Images";
    boost::filesystem::path trainingDirectory(trainingDatasetPath);
    int numTrainingFiles = file_utilities::countFilesInDirectory(trainingDirectory);

    const std::string testDatasetPath = path_to_samples + "/data/Test_Images";
    boost::filesystem::path testDirectory(testDatasetPath);
    int numTestFiles = file_utilities::countFilesInDirectory(testDirectory);

    cv::Mat trainingFeaturesMat = cv::Mat::zeros(numTrainingFiles, numFeatures_, CV_64FC1);
    cv::Mat trainingLabelsMat = cv::Mat::zeros(numTrainingFiles, 1, CV_64FC1);
    cv::Mat testFeaturesMat = cv::Mat::zeros(numTestFiles, numFeatures_, CV_64FC1);
    cv::Mat testLabelsMat = cv::Mat::zeros(numTestFiles, 1, CV_64FC1);

    if (loadClassifierModel_ && file_utilities::exist(svm_file_stream.str().c_str()))
    {
      SVM.load(svm_file_stream.str().c_str());
    }
    else
    {
      if (file_utilities::exist(in_file_stream.str().c_str()) == false ||
          trainingSetFeatureExtraction_)
      {
        std::cout << "Create necessary training matrix" << std::endl;
        std::string prefix = "training_";

        const std::string trainingAnnotationsFile = imageType_ + "training_annotations.txt";
        const std::string trainingAnnotationsFilePath = filesDirectory + trainingAnnotationsFile;

        constructFeaturesMatrix(trainingDirectory, trainingAnnotationsFilePath,
            &trainingFeaturesMat, &trainingLabelsMat);

        normalizeFeaturesAndSaveNormalizationParameters(&trainingFeaturesMat);

        trainingFeaturesMat.convertTo(trainingFeaturesMat, CV_32FC1);
        trainingLabelsMat.convertTo(trainingLabelsMat, CV_32FC1);

        file_utilities::saveFeaturesInFile(trainingFeaturesMat, trainingLabelsMat,
            prefix, trainingFeaturesMatrixFile, trainingLabelsMatrixFile,
            imageType_);
      }
      else
      {
        trainingFeaturesMat = file_utilities::loadFiles(
            in_file_stream.str(), "training_features_mat");
        trainingLabelsMat = file_utilities::loadFiles(
            labels_mat_file_stream.str(), "training_labels_mat");
      }

      // Start Training Process
      std::cout << "Starting training process for the depth images" << std::endl;
      if (USE_OPENCV_GRID_SEARCH_AUTOTRAIN)
      {
        std::cout << "(SVM 'grid search' => may take some time!)" << std::endl;
        SVM.train_auto(trainingFeaturesMat, trainingLabelsMat, cv::Mat(), cv::Mat(),
                       params, 10, CvSVM::get_default_grid(CvSVM::C),
                       CvSVM::get_default_grid(CvSVM::GAMMA),
                       CvSVM::get_default_grid(CvSVM::P),
                       CvSVM::get_default_grid(CvSVM::NU),
                       CvSVM::get_default_grid(CvSVM::COEF),
                       CvSVM::get_default_grid(CvSVM::DEGREE),
                       true);

        params = SVM.get_params();
        std::cout << "Using optimal Parameters" << std::endl;
        std::cout << "degree=" << params.degree << std::endl;
        std::cout << "gamma=" << params.gamma << std::endl;
        std::cout << "coef0=" << params.coef0 << std::endl;
        std::cout << "C=" << params.C << std::endl;
        std::cout << "nu=" << params.nu << std::endl;
        std::cout << "p=" << params.p << std::endl;
      }
      else
      {
        SVM.train(trainingFeaturesMat, trainingLabelsMat, cv::Mat(), cv::Mat(), params);
      }
      SVM.save(svm_file_stream.str().c_str());
      std::cout << "Finished training process" << std::endl;
    }

    if (file_utilities::exist(in_test_file_stream.str().c_str()) == false ||
        testSetFeatureExtraction_)
    {
      std::cout << "Create necessary test matrix" << std::endl;
      std::string prefix = "test_";

      const std::string testAnnotationsFile = imageType_ + "test_annotations.txt";
      const std::string testAnnotationsFilePath = filesDirectory + testAnnotationsFile;

      constructFeaturesMatrix(testDirectory, testAnnotationsFilePath,
          &testFeaturesMat, &testLabelsMat);

      loadNormalizationParametersAndNormalizeFeatures(&testFeaturesMat);

      testFeaturesMat.convertTo(testFeaturesMat, CV_32FC1);
      testLabelsMat.convertTo(testLabelsMat, CV_32FC1);

      file_utilities::saveFeaturesInFile(testFeaturesMat, testLabelsMat,
          prefix, testFeaturesMatrixFile, testLabelsMatrixFile, imageType_);
    }
    else
    {
      testFeaturesMat = file_utilities::loadFiles(
          in_test_file_stream.str(), "test_features_mat");
      testLabelsMat = file_utilities::loadFiles(
          test_labels_mat_file_stream.str(), "test_labels_mat");
    }

    // uncomment to produce the platt probability
    // float prediction;
    // double A, B;
    // for (int ii = 0; ii < testFeaturesMat.rows; ii++)
    // {
      // prediction = SVM.predict(testFeaturesMat.row(ii), true);
      // results.at<double>(ii, 0)= prediction;
    // }
    // sigmoid_train(results, testLabelsMat, &A, &B);
    // std::cout << "A=" << A << std::endl;
    // std::cout << "B=" << B << std::endl;

    // uncomment for ONE_CLASS SVM
    // for (int ii = 0; ii < results.rows; ii++)
     // for (int jj = 0; jj < results.cols; jj++)
      // if(results.at<float>(ii, jj) == 0)
          // results.at<float>(ii, jj) = -1;

    cv::Mat results = cv::Mat::zeros(numTestFiles, 1, CV_64FC1);
    SVM.predict(testFeaturesMat, results);
    // std::cout << "results" << results.size() << std::endl << results <<std::endl <<std::endl;
    file_utilities::saveToFile(results_file_stream.str(), "results", results);
    evaluate(results, testLabelsMat);
  }
}  // namespace pandora_vision

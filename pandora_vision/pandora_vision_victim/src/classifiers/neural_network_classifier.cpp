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
*   Vassilis Choutas  <vasilis4ch@gmail.com>
*********************************************************************/

#include <sstream>
#include <string>

#include <ros/ros.h>

#include "pandora_vision_victim/classifiers/neural_network_classifier.h"

namespace pandora_vision
{
  /**
   * @brief Constructor for the Neural Network Classifier Wrapper Class
   * @param ns[const std::string&] The namespace of the node
   * @param numFeatures[int] The number of input features to the classifier
   * @param datasetPath[const std::string&] The path to the training dataset
   * @param classifierType[const std::string&] The model used by the
   * classifier.
   * @param imageType[const std::string&] The type of input images given to
   * the classifier(RGB or Depth)
   */
  NeuralNetworkClassifier::NeuralNetworkClassifier(const std::string& ns,
      const std::string& datasetPath,
      const std::string& classifierType,
      const std::string& imageType)
    : AbstractClassifier(ns, datasetPath, classifierType,
        imageType)
  {
    ROS_INFO("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Creating Neural "
        "Network training instance");

    XmlRpc::XmlRpcValue layers;
    if (!nh_.getParam(imageType_ + "/layers", layers))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve"
          " the number of neurons in each layer of the network!");
      ros::shutdown();
    }
    int layerNum = static_cast<int>(layers.size());

    // Iterate over the parameter YAML file to get the size for each layer
    // of the ANN.
    cv::Mat layerSizes(1, layerNum, CV_32SC1);
    int ii = 0;

    for (int ii = 0; ii < layerNum ; ii++)
    {
      std::stringstream ss;
      ss << layers[ii];
      ss >> layerSizes.at<signed int>(ii);
    }
    if (layerSizes.at<signed int>(0) != numFeatures_)
    {
      ROS_ERROR_STREAM("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Invalid Input layer neuron number!");
      ROS_BREAK();
    }

    // Get the parameters of  the sigmoid function.
    double alpha, beta;

    if (!nh_.getParam(imageType_ + "/alpha", alpha))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]:Could not retrieve"
          " alpha parameter for sigmoid function!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]:Setting alpha value"
          " to 1!");
      alpha = 1;
    }

    if (!nh_.getParam(imageType_ + "/beta", beta))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve"
          " beta parameter for the sigmoid function!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting beta value"
          " to 1!");
      beta = 1;
    }

    std::string trainingAlgorithm;

    if (!nh_.getParam(imageType_ + "/training_algorithm", trainingAlgorithm))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve"
          " the type of the training algorithm for the neural Network!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Using the standard"
          " back propagation algorithm!");
      trainingAlgorithm = std::string("BackPropagation");
      NeuralNetworkParams_.train_method = cv::ANN_MLP_TrainParams::BACKPROP;
    }

    double learningRate, bpMomentScale;
    // Parse the learning rate parameter
    if (!nh_.getParam(imageType_ + "/learning_rate", learningRate))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve"
          " the learning rate for the training procedure!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting the learning"
          " rate to 0.1");
      learningRate = 0.1;
    }

    if (!nh_.getParam(imageType_ + "/momentum_scale", bpMomentScale))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve"
          " the strength of the momentum term!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting the momentum"
          " term to 0(the feature will be disabled)");
      bpMomentScale = 0.1;
    }

    // Get the maximum number of iterations for the training of the network.
    int maxIter;
    if (!nh_.getParam(imageType_ + "/maximum_iter", maxIter))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve"
          " the maximum number number of training iterations!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting its value"
          " to 1000 iterations.");
      maxIter = 1000;
    }

    // Get the maximum number of iterations for the training of the network.
    double epsilon;
    if (!nh_.getParam(imageType_ + "/epsilon", epsilon))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve"
          " the epsilon value for the error change between iterations!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting its value"
          " to 0.01!");
      epsilon = 0.01;
    }

    // TO DO(Vassilis Choutas): Add RLProp parameters
    // If the training algorithm is not the back propagation algorithm
    // then we must get the rest of the parameters for RPROP algorithm.
    if (trainingAlgorithm.compare("BackPropagation") != 0)
    {
      NeuralNetworkParams_.train_method= cv::ANN_MLP_TrainParams::RPROP;
    }
    else
    {
      NeuralNetworkParams_.train_method= cv::ANN_MLP_TrainParams::BACKPROP;
      NeuralNetworkParams_.bp_dw_scale = learningRate;
      NeuralNetworkParams_.bp_moment_scale = bpMomentScale;
    }
    // Initialize the training algorithm's termination criteria.
    NeuralNetworkParams_.term_crit =
      cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIter, epsilon);

    // Initialize the pointer to the Neural Network Classifier object.
    classifierPtr_.reset(new CvANN_MLP());

    // Create the Neural Network with the specified topology.
    classifierPtr_->create(layerSizes, CvANN_MLP::SIGMOID_SYM,
        alpha, beta);

    ROS_INFO("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Successfully created "
        "Neural Network Classifier Object!");
  }  // End of NeuralNetworkClassifier Constructor

  /**
   * @brief Destructor
   */
  NeuralNetworkClassifier::~NeuralNetworkClassifier()
  {
    ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Destroying Neural Network training instance");
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
  bool NeuralNetworkClassifier::train(const cv::Mat& trainingSetFeatures, const cv::Mat& trainingSetLabels,
      const std::string& classifierFileDest)
  {
    if (trainingSetFeatures.empty())
    {
      ROS_ERROR("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: The features matrix is empty!");
      return false;
    }
    if (trainingSetLabels.empty())
    {
      ROS_ERROR("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: The labels matrix is empty!");
      return false;
    }

    classifierPtr_->train(trainingSetFeatures, trainingSetLabels,
        cv::Mat(), cv::Mat(), NeuralNetworkParams_, CvANN_MLP::NO_INPUT_SCALE + CvANN_MLP::NO_OUTPUT_SCALE);
    classifierPtr_->save(classifierFileDest.c_str());
    return true;
  }

  /**
   * @brief Validates the resulting classifier using the given features
   * extracted from the test set.
   * @param testSetFeatures[const cv::Mat&] The test set features matrix
   * @param validationResults[cv::Mat*] The results for the test set.
   * @return void
   */
  void NeuralNetworkClassifier::validate(const cv::Mat& testSetFeatures, cv::Mat* validationResults)
  {
    float returnVal = classifierPtr_->predict(testSetFeatures, *validationResults);
    for (int i = 0; i < validationResults->rows; ++i)
      validationResults->at<float>(i) = validationResults->at<float>(i) > 0 ? 1.0f : -1.0f;
    return;
  }
}  // namespace pandora_vision


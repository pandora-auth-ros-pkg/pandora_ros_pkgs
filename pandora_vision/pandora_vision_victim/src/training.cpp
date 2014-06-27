/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/training.h"

namespace pandora_vision
{
  /**
@brief Constructor
**/
  SvmTraining::SvmTraining(int _num_files, int _num_feat) : _nh()
  {
    num_files = _num_files;
    num_feat = _num_feat;
    
    params.svm_type = CvSVM::C_SVC;
    params.kernel_type = CvSVM::LINEAR;
    params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    
    package_path = ros::package::getPath("pandora_vision_victim");
    ROS_INFO("[victim_node] : Created Svm training instance");
    
  }
  
  /**
@brief Second constructor
**/
  SvmTraining::SvmTraining(int _num_files, int _rgb_num_files, int _depth_num_files) : _nh()
  {
    num_files = _num_files;
    rgb_num_feat = _rgb_num_files;
    depth_num_feat = _depth_num_files;
    
    params.svm_type = CvSVM::C_SVC;
    params.kernel_type = CvSVM::LINEAR;
    params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    
    package_path = ros::package::getPath("pandora_vision_victim");
    ROS_INFO("[victim_node] : Created Svm training instance");
    
  }

  /**
@brief Destructor
*/
  SvmTraining::~SvmTraining()
  {
    ROS_DEBUG("[victim_node] : Destroying Svm training instance");
  }


 /**
* @brief This method constructs the rgb training matrix
* to be used for the training
* @param [std::string] file name to save the extracted training matrix
* @param [int] type, Value that indicates, if we train depth subsystem,
* or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
* @return void
*/
  void SvmTraining::constructTrainingMatrix(std::string file_name, int type)
  {
    cv::Mat img;
    std::vector<double> _featureVector;

    training_mat = cv::Mat::zeros(num_files, num_feat, CV_64FC1);
    labels_mat = cv::Mat::zeros(num_files, 1, CV_64FC1);

    std::stringstream img_name;
    for (int i=0; i< num_files; i++)
    {
      if(i<NUM_OF_POSITIVE_SAMPLES)
        img_name << path_to_samples << "/data/"
            << "Positive_Images/positive" << i+1 << ".jpg";
      else
        img_name << path_to_samples << "/data/"
            << "Negative_Images/negative" << i + 1 - NUM_OF_POSITIVE_SAMPLES << ".jpg";
        
      std::cout<<img_name.str()<<std::endl;
        
      img = cv::imread(img_name.str());
      if (!img.data)
      {
       std::cout << "Error reading file " << img_name << std::endl;
       return;
      }
      
      cv::Size size(640,480);
      cv::resize(img,img,size);
      
      if(type == 1)
      {
        _rgbSystem.extractRgbFeatures(img);
        _featureVector=_rgbSystem.getRgbFeatureVector();
      }
      
      else
      {
        _depthSystem.extractDepthFeatures(img);
        _featureVector=_depthSystem.getDepthFeatureVector();
      }
      
      std::cout<<"Feature vector of image " << img_name.str() << " "
          <<_featureVector.size()<<std::endl;

      for (int j = 0; j < _featureVector.size(); j++)
        training_mat.at<double>(i,j)=_featureVector[j];
                

      if(i < NUM_OF_POSITIVE_SAMPLES)
        labels_mat.at<double>(i,0)= 1.0;
      
      else
        labels_mat.at<double>(i,0) = -1.0;

        
      ///Empty stringstream for next image
      img_name.str("");
      
    }
    
    std::stringstream training_mat_file_stream;
    training_mat_file_stream << package_path << "/data/" << file_name;
    cv::FileStorage fs(training_mat_file_stream.str(), cv::FileStorage::WRITE);

    if (!fs.isOpened())
      fs.open(training_mat_file_stream.str(), cv::FileStorage::WRITE);
    
    fs << "training_mat" << training_mat;
    fs.release();
    
    std::cout << training_mat_file_stream.str() << std::endl <<
        " " << training_mat.size()<< std::endl;
        
    std::stringstream labels_mat_file_stream;
    labels_mat_file_stream << package_path << "/data/" << "labels_"+file_name;
    cv::FileStorage fos(labels_mat_file_stream.str(), cv::FileStorage::WRITE);

    if (!fos.isOpened())
      fos.open(labels_mat_file_stream.str(), cv::FileStorage::WRITE);
    
    fos << "labels_mat" << labels_mat;
    fos.release();
    
    std::cout << labels_mat_file_stream.str() << std::endl <<
        " " << labels_mat.size()<< std::endl;
       
  }
  
  
  /**
*@brief Function that implements the training for the subsystems
* according to the given training sets. It applies svm and extracts
* a suitable model
* @param [int] type, Value that indicates, if we train depth subsystem,
* or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
* @return void
*/
  void SvmTraining::trainSubSystem(int type)
  {
    std::string training_matrix_file_path;
    std::string labels_matrix_file_path;
    std::stringstream in_file_stream;
    std::stringstream labels_mat_file_stream;
    std::stringstream svm_file_stream;

    switch(type){
      case 1:
        /// Train only rgb subsystem
        training_matrix_file_path = "rgb_training_matrix.xml";
      
        in_file_stream << package_path << "/data/" << training_matrix_file_path;
        labels_mat_file_stream << package_path << "/data/" << "labels_" +training_matrix_file_path;
        svm_file_stream << package_path << "/data/" << "rgb_svm_classifier.xml";

        if(exist(in_file_stream.str().c_str()) == false){
          std::cout << "Create necessary training matrix" <<std::endl;
          constructTrainingMatrix(training_matrix_file_path, type);
        }
      
        loadFiles(in_file_stream.str(),labels_mat_file_stream.str());
        std::cout << "Starting training process for the rgb images" << std::endl;
        SVM.train(training_mat, labels_mat, cv::Mat(), cv::Mat(), params);
        SVM.save(svm_file_stream.str().c_str());
        std::cout << "Finished training process" << std::endl;
        break;
        
      case 2:
        /// Train only depth subsystem
        training_matrix_file_path = "depth_training_matrix.xml";
        
        in_file_stream << package_path << "/data/" << training_matrix_file_path;
        labels_mat_file_stream << package_path << "/data/" << "labels_"+training_matrix_file_path;
        svm_file_stream << package_path << "/data/" << "depth_svm_classifier.xml";

        if(exist(in_file_stream.str().c_str()) == false){
          std::cout << "Create necessary training matrix" <<std::endl;
          constructTrainingMatrix(training_matrix_file_path, type);
        }
        
        loadFiles(in_file_stream.str(),labels_mat_file_stream.str());
        std::cout << "Starting training process for the depth images" << std::endl;
        SVM.train(training_mat, labels_mat, cv::Mat(), cv::Mat(), params);
        SVM.save(svm_file_stream.str().c_str());
        std::cout << "Finished training process" << std::endl;
        break;
        
      case 3:
        /// Train both subsystems
        training_matrix_file_path = "rgb_training_matrix.xml";
        
        std::stringstream in_file_stream;
        in_file_stream << package_path << "/data/" << training_matrix_file_path;
        labels_mat_file_stream << package_path << "/data/" << "labels_"+training_matrix_file_path;
        svm_file_stream << package_path << "/data/" << "rgb_svm_classifier.xml";

        if(exist(in_file_stream.str().c_str()) == false){
          std::cout << "Create necessary rgb training matrix" <<std::endl;
          num_feat = rgb_num_feat;
          constructTrainingMatrix(training_matrix_file_path, 1);
        }
        
        loadFiles(in_file_stream.str(),labels_mat_file_stream.str());
        std::cout << "Starting training process for the rgb images" << std::endl;
        SVM.train(training_mat, labels_mat, cv::Mat(), cv::Mat(), params);
        SVM.save(svm_file_stream.str().c_str());
        std::cout << "Finished training process for rgb system" << std::endl;
        
        training_matrix_file_path = "depth_training_matrix.xml";
        std::stringstream in1_file_stream,labels1_mat_file_stream,svm1_file_stream;

        in1_file_stream << package_path << "/data/" << training_matrix_file_path;
        labels1_mat_file_stream << package_path << "/data/" << "labels_"+training_matrix_file_path;
        svm1_file_stream << package_path << "/data/" << "depth_svm_classifier.xml";
        
        if(exist(in1_file_stream.str().c_str()) == false){
          std::cout << "Create necessary depth training matrix" <<std::endl;
          num_feat = depth_num_feat;
          constructTrainingMatrix(training_matrix_file_path, 2);
        }
        
        loadFiles(in1_file_stream.str(),labels1_mat_file_stream.str());
        std::cout << "Starting training process for the depth images" << std::endl;
        SVM.train(training_mat, labels_mat, cv::Mat(), cv::Mat(), params);
        SVM.save(svm1_file_stream.str().c_str());
        std::cout << "Finished training process" << std::endl;
        break;
    }
  }
  
   /**
*@brief Function that loads the necessary files for the training
* @param [std::string] training_mat_file, name of the file that contains the training data
* @param [std::string] labels_mat_file, name of the file that contains the labels of each class
* of the training data
* @return void
*/
  void SvmTraining::loadFiles(std::string training_mat_file, std::string labels_mat_file)
  {
    cv::FileStorage fs1,fs2;
    cv::Mat temp1,temp2;
    fs1.open(training_mat_file, cv::FileStorage::READ);
    fs1["training_mat"] >> temp1;
    fs2.open(labels_mat_file, cv::FileStorage::READ);
    fs2["labels_mat"] >> temp2;
    fs1.release();
    fs2.release();
    if (temp1.data && temp2.data)
      {
       std::cout << "files uploaded successfully" << std::endl;
       std::cout << training_mat_file << temp1.size() << std::endl;;
       std::cout << labels_mat_file << temp2.size() << std::endl;
      }
    
    training_mat = temp1.clone();
    labels_mat = temp2.clone();
    training_mat.convertTo(training_mat, CV_32FC1);
    labels_mat.convertTo(labels_mat, CV_32FC1);
  }

  /**
*@brief Function that checks if a file exists
*@param name Name of file to check if exists
*@return true if the file was found, and false if not
*/
  bool SvmTraining::exist(const char *name)
  {
    std::ifstream file(name);
    if(!file) //if the file was not found, then file is 0, i.e. !file=1 or true
        return false; //the file was not found
    else //if the file was found, then file is non-0
        return true; //the file was found
  }
  
}// namespace pandora_vision

int main(int argc, char** argv)
{
  ros::init(argc, argv, "victim_train_node");
  
  /// num_files = 9500 and rgb_num_feat = 121 depth_num_feat=103
  int num_files, num_feat, type, rgb_num_feat, depth_num_feat;
  std::cout << "Choose type of training: 1 rgb ,2 depth,3 both"<<std::endl;
  std::cin >> type;
  std::cout << "Add total number of images to be trained :" <<std::endl;
  std::cin >> num_files;
  
  if(type == 1 || type == 2){
    std::cout << "Add total number of features required for your subsystem :" << std::endl;
    std::cin >> num_feat;
    pandora_vision::SvmTraining victim_trainer(num_files, num_feat);
    std::cout << "Add absolute path, where your samples are stored "<<std::endl;
    std::cin >> victim_trainer.path_to_samples;
    victim_trainer.trainSubSystem(type);
  }
  if(type == 3){
    std::cout << "Add total number of features required for rgb subsystem :" << std::endl;
    std::cin >> rgb_num_feat;
    std::cout << "Add total number of features required for depth subsystem :" << std::endl;
    std::cin >> depth_num_feat;
    pandora_vision::SvmTraining victim_trainer(num_files, rgb_num_feat, depth_num_feat);
    std::cout << "Add absolute path, where your samples are stored "<<std::endl;
    std::cin >> victim_trainer.path_to_samples;
    victim_trainer.trainSubSystem(type);
  }

  ros::spin();
  return 0;
}

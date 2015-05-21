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
 * Authors:
 *   Chamzas Konstantinos <chamzask@gmail.com>
 *********************************************************************/
 
#include "pandora_geotiff/qr_csv_creator.h"
namespace pandora_geotiff{

  QrCsvCreator::QrCsvCreator()
  
  {
    gotData_ = false;
    missionNamePrefix_ = std::string("/RRL_2015_PANDORA_");
    }
  
   void QrCsvCreator::getQrsData(){
  
        ros::NodeHandle nh_("~");
        pandora_data_fusion_msgs::QrCsvSrv dataFusionQrSrv;
        
        ros::ServiceClient service_client_ = nh_.serviceClient<pandora_data_fusion_msgs::QrCsvSrv>("Mavrodis");
        if (!service_client_.call(dataFusionQrSrv)) {
          ROS_ERROR("Cannot get Qrs-Content , service %s failed",
           service_client_.getService().c_str());
          return;
        }
        qrs_ = dataFusionQrSrv.response.qrs;
        ROS_INFO("QRS_CONTENTS_SAVED SUCCESEFULLY");
        gotData_ = true;
      }
    
    
  std::string QrCsvCreator::getDateAndTime(){
  
    time_t t = time(0); // get time now
    struct tm * now = localtime( & t );
    char buf[20];
    std::stringstream ss;
    
    strftime(buf, sizeof(buf), "%F", now);
    
    ss << buf << "; ";
    
    strftime(buf, sizeof(buf), "%T", now);
    
    ss << buf;
    
    std::string str = ss.str();
  
    return str;
  
  }
  
  std::string QrCsvCreator::getQrTime(time_t qrTime){
    
    struct tm * now = localtime( & qrTime );
    char buf[10];
    std::stringstream ss;
    strftime(buf, sizeof(buf), "%T", now);
    ss << buf;
    
    std::string str = ss.str();
  
    return str;
  }
  
  void QrCsvCreator::generateQrCsv(std::string missionName){
  
    if (gotData_){
      std::string filenameString(missionNamePrefix_);
      filenameString.append(missionName);
      filenameString.append("_qr.csv");
    
      passwd* pw = getpwuid(getuid());
      std::string filepath(pw->pw_dir);
      filepath = filepath.append("/Desktop/");
      filepath.append(filenameString);
    
      std::ofstream csvFile;
      csvFile.open (filepath.c_str());
    
      csvFile << "PANDORA AUTh, Greece" << std::endl;
      csvFile << getDateAndTime() << std::endl;
      csvFile << missionName <<std::endl;
      for (int i = 0 ; i< qrs_.size(); i++){
        csvFile << i+1 << ";" << qrs_[i].header.stamp << ";" << qrs_[i].content << ";" << qrs_[i].x << ";" << qrs_[i].y << std::endl;
        }
      csvFile.close();   
      }
    }
}//namespace pandora_geotiff

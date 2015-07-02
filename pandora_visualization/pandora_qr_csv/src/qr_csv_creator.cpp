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
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions of source code must retain the above copyright
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
 *   Sideris Konstantinos <siderisk@auth.gr>
 *********************************************************************/

#include <string>
#include <ctime>

#include "pandora_qr_csv/qr_csv_creator.h"


namespace pandora_qr_csv
{
  QrCsvCreator::QrCsvCreator()
  {
    std::string nodeName = ros::this_node::getName();
    qrReceived_ = false;

    /**
     * Configuration.
     */

    nh_.param(nodeName + "/team/name", teamName_, std::string("PANDORA"));
    nh_.param(nodeName + "/team/country", country_, std::string("Greece"));

    nh_.param(nodeName + "/robot/name", robotName_, std::string("Pandora"));
    nh_.param(nodeName + "/robot/mode", robotMode_, std::string("A"));

    nh_.param(nodeName + "/csv/prefix", csvPrefix_, std::string("RC2015"));
    nh_.param(nodeName + "/csv/suffix", csvSuffix_, std::string("_qr.csv"));
    nh_.param(nodeName + "/csv/directoryToSave", directoryToSave_, std::string("/home/pandora"));

    nh_.param(nodeName + "/services/objects", objectsServiceName_, std::string("data_fusion_geotiff"));
    nh_.param(nodeName + "/services/csv", qrCsvServiceName_, std::string("qr_csv/createCSV"));

    /**
     * Objects Service.
     */

    objectClient_ = nh_.serviceClient<pandora_data_fusion_msgs::GetGeotiff>(objectsServiceName_);

    /**
     * QRs to CSV service.
     */

    createCsvService_ = nh_.advertiseService(qrCsvServiceName_, &QrCsvCreator::handleRequest, this);
  }

  void QrCsvCreator::getQrsData()
  {
    pandora_data_fusion_msgs::GetGeotiff dataFusionSrv;

    ROS_INFO("Calling %s", objectClient_.getService().c_str());

    if (!objectClient_.call(dataFusionSrv))
    {
      ROS_ERROR("Service %s has failed", objectClient_.getService().c_str());
      return;
    }

    ROS_INFO("Service %s responded.", objectClient_.getService().c_str());
    qrReceived_ = true;

    qrs_ = dataFusionSrv.response.qrs;
  }

  bool QrCsvCreator::handleRequest(createCSV::Request &req, createCSV::Response &res)
  {
    ROS_INFO("CSV creation was requested.");
    ROS_INFO("Name of the mission: %s", req.missionName.data.c_str());

    std::string currentDate = this -> getCurrentDate();
    std::string currentTime = this -> getCurrentTime();

    this -> getQrsData();

    if (!qrReceived_) {
      ROS_ERROR("QR data were not received. Aborting...");
      res.result.data = false;
      return true;
    }

    this -> generateQrCsv(req.missionName.data, currentDate, currentTime);
    res.result.data = true;

    return true;
  }

  std::string QrCsvCreator::getCurrentTime()
  {
    time_t t = time(0);
    struct tm *now = localtime(&t);

    char buf[20];
    std::stringstream ss;

    strftime(buf, sizeof(buf), "%T", now);
    ss << buf;

    std::string timeStamp = ss.str();

    ROS_INFO("Current time: %s", timeStamp.c_str());

    return timeStamp;
  }

  std::string QrCsvCreator::getCurrentDate()
  {
    time_t t = time(0);
    struct tm *now = localtime(&t);

    char buf[20];
    std::stringstream ss;

    strftime(buf, sizeof(buf), "%F", now);
    ss << buf;

    std::string dateStamp = ss.str();

    ROS_INFO("Current date: %s", dateStamp.c_str());

    return dateStamp;
  }

  void QrCsvCreator::generateQrCsv(std::string &missionName, std::string &currentDate, std::string &currentTime)
  {
    ROS_INFO("Creating CSV file.");

    std::string filename(csvPrefix_);
    filename.append("_");
    filename.append(teamName_);
    filename.append("_");
    filename.append(missionName);
    filename.append(csvSuffix_);

    passwd* pw = getpwuid(getuid());
    std::string filepath(pw->pw_dir);
    filepath = filepath.append(directoryToSave_);
    filepath.append(filename);

    std::ofstream csvFile;
    csvFile.open(filepath.c_str());

    csvFile << "\"qr codes\"" << std::endl;
    csvFile << "\"1.1\"" << std::endl;

    csvFile << "\"" + teamName_ + "\"" << std::endl;
    csvFile << "\"" + country_ + "\"" << std::endl;

    csvFile << "\"" + currentDate + "\"" << std::endl;
    csvFile << "\"" + currentTime + "\"" << std::endl;
    csvFile << "\"" + missionName + "\"" << std::endl;

    // A blank line is needed.
    csvFile << std::endl;

    // Body descripion.
    csvFile << "id,time,text,x,y,z,robot,mode" << std::endl;

    for (int i = 0 ; i< qrs_.size(); i++)
    {
      csvFile << qrs_[i].id << "," << currentTime
                            << "," << qrs_[i].content
                            << "," << qrs_[i].qrPose.pose.position.x
                            << "," << qrs_[i].qrPose.pose.position.y
                            << "," << qrs_[i].qrPose.pose.position.z
                            << "," << robotName_
                            << "," << robotMode_ << std::endl;
    }
    ROS_INFO("Saving file to %s.", filepath.c_str());

    csvFile.close();
  }
}  // namespace pandora_qr_csv

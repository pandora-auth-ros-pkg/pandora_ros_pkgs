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
 *   Sideris Konstantinos <siderisk@auth.gr>
 *********************************************************************/


#ifndef PANDORA_QR_CSV_QR_CSV_CREATOR_H
#define PANDORA_QR_CSV_QR_CSV_CREATOR_H

#include <stdlib.h>
#include <vector>
#include <string>
#include <pwd.h>
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>

#include <ros/ros.h>

#include "pandora_qr_csv/createCSV.h"
#include "pandora_data_fusion_msgs/QrInfo.h"
#include "pandora_data_fusion_msgs/HazmatInfo.h"
#include "pandora_data_fusion_msgs/GetGeotiff.h"


namespace pandora_qr_csv
{
  class QrCsvCreator
  {
    public:
      /**
       * @brief QrCsvCreator constructor.
       */

      QrCsvCreator();

      /**
       * @brief Creates the csv file with the QRs.
       *
       * @param missionName [&std::string] The name of the mission given by the
       *                                   service client.
       * @param startDate   [&std::string] The date of the RoboCup mission.
       * @param startTime   [&std::string] The current time of the csv request.
       */

      void generateQrCsv(std::string &missionName, std::string &currentDate, std::string &currentTime);

      bool handleRequest(createCSV::Request &req, createCSV::Response &res);

    private:
      //!< ROS node handler.
      ros::NodeHandle nh_;

      //!< ROS Service to hanlde the requests.
      ros::ServiceServer createCsvService_;

      //!< Client for Data Fusion's object service.
      ros::ServiceClient objectClient_;

      //!< The name of the objects service.
      std::string objectsServiceName_;

      //!< The name of the qr to csv service.
      std::string qrCsvServiceName_;

      //!< The name of the team that is competing.
      std::string teamName_;

      //!< The country that the team represents.
      std::string country_;

      //!< The name of the robot.
      std::string robotName_;

      //!< The mode of the robot. A for autonomous, T for teleoperated.
      std::string robotMode_;

      //!< The prefix of the csv file required by the RoboCup competition.
      std::string csvPrefix_;

      //!< The suffix of the csv file required by the RoboCup competition.
      std::string csvSuffix_;

      //!< The directory to save the file.
      std::string directoryToSave_;

      //!< Flag
      bool objectsReceived_;

      //!< QRs received from DataFusion.
      std::vector<pandora_data_fusion_msgs::QrInfo> qrs_;

      //!< Obstacles received from DataFusion.
      std::vector<pandora_data_fusion_msgs::ObstacleInfo> obstacles_;

      void getObjects();

      std::string getObstaclesName(unsigned int code);

      std::string getCurrentDate();

      std::string getCurrentTime();

      std::string rosTimeToLocal(ros::Time aTime);
  };
}  // namespace pandora_qr_csv

#endif  // PANDORA_QR_CSV_QR_CSV_CREATOR_H



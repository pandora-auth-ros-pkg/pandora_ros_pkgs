/******************************************************************************
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
*   Protopapas Marios <protopapas_marios@hotmail.com>
*   Manos Tsardoulias <etsardou@gmail.com>
*********************************************************************/

#include "pandora_vision_annotator/annotator_tools.h"

namespace pandora_vision
{ 
  int ImgAnnotations::annPerImage = 0;
  bool ImgAnnotations::secondpoint = false;
  int ImgAnnotations::originalHeight = 480;
  int ImgAnnotations::originalWidth = 640;
  std::vector<annotation> ImgAnnotations::annotations;
  std::ofstream ImgAnnotations::outFile;
  std::ifstream ImgAnnotations::inFile;
  annotation ImgAnnotations::temp;
  
  /**
  @brief function that deletes annotations file
  @param filename [const std::string&] the path of annotations file
  @return void
  **/
  void ImgAnnotations::removeFile(const std::string& filename)
  {
    bool flag = is_file_exist(filename.c_str());
    if(flag)
      remove(filename.c_str());
  }

  /**
  @brief function that deletes annotations of current frame from file
  @param filename [const std::string&] the path of annotations file
  @param frame [const std::string&] the frame name
  @return void
  **/
  void ImgAnnotations::deleteFromFile(const std::string&filename, const std::string& frame)
  {
    std::string line;
    std::string package_path = ros::package::getPath("pandora_vision_annotator");
    std::stringstream tempFile;
    tempFile << package_path << "/data/temp.txt";
    bool deleted = false;
    int length = frame.length();
    inFile.open(filename.c_str());
    outFile.open(tempFile.str().c_str());
    if(!inFile)
    {
      ROS_ERROR("cannot load file");
    }
    while(std::getline(inFile, line))
    {

      if(line.substr(0, length) != frame)
      {
        outFile << line << std::endl;
        ROS_INFO_STREAM(line);
      }

      else
        deleted = true;

      if(deleted)
      {
        ROS_INFO_STREAM("Annotation for " << frame <<" deleted from file");
      }
    }

      inFile.close();
      outFile.close();
      remove(filename.c_str());
      rename(tempFile.str().c_str(), filename.c_str());
  }
  
  /**
  @brief function that loads last frame index from
  @param filename [const std::string&] the path of annotations file
  @param index [int&] the index to be loaded
  @return void
  **/
  void ImgAnnotations::getLastFrameIndex(const std::string& filename, int* index)
  {
    inFile.open(filename.c_str(), std::ios_base::ate);
    std::string line;
    int length = 0;
    char c ='\0';

    if (!inFile)
    {
      ROS_ERROR("cannot load file");
      return;
    } 

    length = inFile.tellg();
    if(length == 0)
    {
      *index = -1;
      inFile.close();
    }

    else
    {

      for(int ii = length - 2; ii > 0; ii--)
      {
        inFile.seekg(ii);
        c = inFile.get();
        if( c =='\r' || c == '\n' )
          break;
      }
      std::getline(inFile, line);
      std::string package_path = ros::package::getPath("pandora_vision_annotator");
      std::stringstream temp;
      temp << line;
      std::string imName;
      getline (temp, imName, ',');
      unsigned s = imName.find_last_of(".");
      imName = imName.substr(0, s);
      s = imName.find_last_of("e");
      imName = imName.substr(s + 1);
      *index= atoi(imName.c_str());
      inFile.close();
    }
  }

  /**
  @brief function that loads annotations of current frame from file
  @param filename [const std::string&] the path of annotations file
  @param frame [const std::string&] the frame name
  @return void
  **/
  void ImgAnnotations::readFromFile(const std::string& filename, const std::string& frame)
  {
    std::string line, x1, y1, x2, y2;
    inFile.open(filename.c_str());
    int  i = 0;
    int length = frame.length();
    if (!inFile)
    {
      ROS_ERROR("cannot load file");
      return;
    }

    else
    {
      if(inFile.is_open())
      {
        ROS_INFO("loading from file");
        while(std::getline(inFile, line))
        {  

          if(line.substr(0, length) == frame)
          {ROS_INFO_STREAM("LENGTH" << length << " " << line.substr(0, length));

            std::stringstream ss(line);
            ROS_INFO("stringstream %s", ss.str().c_str());
            getline (ss, ImgAnnotations::temp.imgName, ',');
            getline (ss, ImgAnnotations::temp.category, ',');
            getline (ss, x1, ',');
            getline (ss, y1, ',');
            getline (ss, x2, ',');
            if(ImgAnnotations::temp.category == "Hazmat")
            {
              getline (ss, y2, ',');
              getline (ss, ImgAnnotations::temp.type);
            }           
            else
              getline (ss, y2);
            ImgAnnotations::temp.x1 = floor((640.0f / ImgAnnotations::originalWidth) * 
                                      atoi(x1.c_str()));
            ImgAnnotations::temp.y1 = floor((480.0f / ImgAnnotations::originalHeight) *
                                      atoi(y1.c_str()));
            ImgAnnotations::temp.x2 = floor((640.0f / ImgAnnotations::originalWidth) *
                                      atoi(x2.c_str()));
            ImgAnnotations::temp.y2 = floor((480.0f / ImgAnnotations::originalHeight) *
                                      atoi(y2.c_str()));

            ROS_INFO_STREAM("Loading Annotation no: " << i+1 <<" for "<< frame << "\n");
            ROS_INFO_STREAM(ImgAnnotations::temp.imgName << ","
                            << ImgAnnotations::temp.category << ","
                            << ImgAnnotations::temp.x1 << ","
                            << ImgAnnotations::temp.y1<< ","
                            << ImgAnnotations::temp.x2 << ","
                            << ImgAnnotations::temp.y2 << "\n" );
            ImgAnnotations::annotations.push_back(temp);
            i++;
        
          }          
        }
      }
    }
    inFile.close(); 
  }

  /**
  @brief function that writes annotations of current frame to file
  @param filename [const std::string&] the path of annotations file
  @return void
  **/
  void ImgAnnotations::writeToFile(const std::string& filename)
  {
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
    if (!outFile)
    {
      ROS_ERROR("cannot load file");
      return;
    }
    else
    {  
      if(outFile.is_open())
      {
        ROS_INFO("Writing to file" );
        for (unsigned int i = 0; i < annotations.size(); i++)
        {
          ImgAnnotations::temp.imgName = ImgAnnotations::annotations[i].imgName;
          ImgAnnotations::temp.category = ImgAnnotations::annotations[i].category;
          ImgAnnotations::temp.x1 = floor((ImgAnnotations::originalWidth / 640.0f) *
                                    ImgAnnotations::annotations[i].x1);
          ImgAnnotations::temp.x2 = floor((ImgAnnotations::originalWidth / 640.0f) *
                                    ImgAnnotations::annotations[i].x2);
          ImgAnnotations::temp.y1 = floor((ImgAnnotations::originalHeight / 480.0f) *
                                    ImgAnnotations::annotations[i].y1);
          ImgAnnotations::temp.y2 = floor((ImgAnnotations::originalHeight / 480.0f) *
                                    ImgAnnotations::annotations[i].y2);
          ImgAnnotations::temp.type = ImgAnnotations::annotations[i].type;
          
          outFile << ImgAnnotations::temp.imgName << ","
                  << ImgAnnotations::temp.category << ","
                  << ImgAnnotations::temp.x1 << ","
                  << ImgAnnotations::temp.y1 << ","
                  << ImgAnnotations::temp.x2 << ","
                  << ImgAnnotations::temp.y2;
          if (ImgAnnotations::temp.category == "Hazmat")
            outFile << ","<< ImgAnnotations::temp.type;
          outFile << std::endl;
          qDebug("%s %s %d %d %d %d\n", ImgAnnotations::temp.imgName.c_str(),
                  ImgAnnotations::temp.category.c_str(),
                  ImgAnnotations::temp.x1,
                  ImgAnnotations::temp.y1,
                  ImgAnnotations::temp.x2,
                  ImgAnnotations::temp.y2);
         }
      }
    }
    outFile.close();
  }
 
  /**
  @brief function that writes the msg header of current frame to file
  @param filename [const std::string&] the path of annotations file
  @param msg [const std_msg::Header&] the msg header
  @return void
  **/
  void ImgAnnotations::writeToFile(const std::string& filename, const std_msgs::Header& msg )
  {
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
    if (!outFile)
    {
      ROS_ERROR("cannot load file");
      return;
    }
    else
    {
      if(outFile.is_open())
      {
        ROS_INFO("Writing to file" );
        outFile << msg.seq << "," << msg.stamp << "," << msg.frame_id << std::endl;
      }
    }
    outFile.close();
  }

  /**
  @brief function that checks if file exists 
  @param filename [const char*] the name of the file
  @return void
  **/
  bool ImgAnnotations::is_file_exist(const char *fileName)
  {
    std::ifstream infile(fileName);
    return infile.good();
  }

  /**
  @brief function that sets annotations of current frame 
   @param ImgName [const std::string&] the name of the image
  @param category [const std::string&] the annotation category
  @param x [int] the x coordinate
  @param y [int] the y coordinate
  @return void
  **/
  void ImgAnnotations::setAnnotations(const std::string &imgName, const std::string &category, int x, int y)
  {
    if(secondpoint)
    {
      ImgAnnotations::temp.x2 = x;
      ImgAnnotations::temp.y2 = y;
      ImgAnnotations::annotations.push_back(temp);
      qDebug(" Annotations in current frame %ld", ImgAnnotations::annotations.size());
      ImgAnnotations::secondpoint = false;
    }
    else
    {
      ImgAnnotations::temp.imgName = imgName;
      ImgAnnotations::temp.category = category;
      ImgAnnotations::temp.x1 = x;
      ImgAnnotations::temp.y1 = y;
      ImgAnnotations::secondpoint = true;
    }
  }
  
  /**
  @brief function that sets annotations of current frame for hazmat 
  @param ImgName [const std::string&] the name of the image
  @param category [const std::string&] the annotation category
  @param x [int] the x coordinate
  @param y [int] the y coordinate
  @param type [const std::string&] the hazmat type
  @return void
  **/
  void ImgAnnotations::setAnnotations(const std::string &imgName, 
                                      const std::string &category, 
                                      int x, int y, const std::string& type)
  {
    if(secondpoint)
    {
      ImgAnnotations::temp.x2 = x;
      ImgAnnotations::temp.y2 = y;
      ImgAnnotations::temp.type = type;
      ImgAnnotations::annotations.push_back(temp);
      qDebug(" Annotations in current frame %ld", ImgAnnotations::annotations.size());
      ImgAnnotations::secondpoint = false;
    }
    else
    {
      ImgAnnotations::temp.imgName =imgName;
      ImgAnnotations::temp.category = category;
      ImgAnnotations::temp.x1 = x;
      ImgAnnotations::temp.y1 = y;
      ImgAnnotations::secondpoint = true;
    }
  }
  
  /**
  @brief function that sets original Image dimensions 
  @param  width [int] the image's original width
  @param height [int] the image's original img_height
  @return void
  **/
  void ImgAnnotations::setOriginalImgDimensions(int width, int height)
  {
    ImgAnnotations::originalWidth = width;
    ImgAnnotations::originalHeight = height;
    ROS_INFO_STREAM("save Original img dimensions" << ImgAnnotations::originalWidth 
                                                   << " " << ImgAnnotations::originalHeight);
      
  }
}// namespace pandora_vision

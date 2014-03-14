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
*   Nikos Gountas 
*   Triantafyllos Afouras <afourast@gmail.com>
*********************************************************************/

#include "pandora_interface_diagnostics/interfaces_xml_parser.h"

InterfacesXmlParser::InterfacesXmlParser() {
    
  std::string interfaceListPackagePath;
  
  ros::NodeHandle nh;
  FILE * filePtr;
  
  if (nh.hasParam("interfaceListPackagePath")) {
    nh.getParam("interfaceListPackagePath", interfaceListPackagePath);
  }  
  else{
    ROS_ERROR("[Interface Tester]: interfaceListPackagePath param was not set");
    exit(0);
  }
  
  boost::shared_ptr<TiXmlDocument> doc(new TiXmlDocument);
  
  std::string fullPath = interfaceListPackagePath + "/interfaces_list.xml";
  filePtr = fopen ( fullPath.c_str(), "r" );
  doc->LoadFile(filePtr); 
  fclose(filePtr);

  TiXmlElement* packagesParent = doc->FirstChildElement("packages");
  
  if(! packagesParent){
    ROS_ERROR("[Interface Tester] : "
      "interfaces_list does not contain packages element") ;
    exit(0);
  }
  
  TiXmlElement*  rosParam_element = 
    packagesParent->FirstChildElement("include_rosparam");

  while(rosParam_element){
    
    std::string param = trim(rosParam_element->GetText());
    std::string packagePath ;
    
    if (nh.hasParam(trim(param))) {
      nh.getParam(param, packagePath);
    }  
    else{
      ROS_ERROR("[Interface Tester] : %s param was not set  ",param.c_str()) ;
      continue;
    }
    
    std::string fullPath = packagePath + "/interfaces.xml";
    
    TiXmlDocument* doc = new TiXmlDocument;
    filePtr = fopen ( fullPath.c_str(), "r" );
    doc->LoadFile( filePtr ); 
    docsVector_.push_back(doc);
    fclose(filePtr);
    rosParam_element = rosParam_element->NextSiblingElement("include_rosparam");

  }
    
};

InterfacesXmlParser::~InterfacesXmlParser(){
  for(int ii=0;ii<docsVector_.size();ii++){
    delete docsVector_[ii]; 
  }
}

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

#ifndef INTERFACEDIAGNOSTICS_H
#define INTERFACEDIAGNOSTICS_H

#include <iostream>
#include "tinyxml.h"
#include <stdio.h>

#include <ros/ros.h>
#include "state_client.h"
#include "interface_tester/tf_monitor.h"
#include "interface_tester/interface_tester.h"

#include "trimming.h"
#include "generic_diagnostic.h"
#include "interfaces_xml_parser.h"

class InterfaceDiagnostics: GenericDiagnostic, StateClient {
  
 public:
  
  InterfaceDiagnostics();
  
  ~InterfaceDiagnostics();

  virtual void startTransition (int newState);
  
 private:

  void nodeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  void nodePublisherDiagnostic(TiXmlElement* nodeElement, 
    diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk);
  void nodeSubscriberDiagnostic(TiXmlElement* nodeElement, 
    diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk);
  void nodeActionServerDiagnostic(TiXmlElement* nodeElement, 
    diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk);
  void nodeActionClientDiagnostic(TiXmlElement* nodeElement, 
    diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk);
  
  void tfPublisherDiagnostic(TiXmlElement* nodeElement, 
    diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk);
  void tfTransformDiagnostic(TiXmlElement* nodeElement, 
    diagnostic_updater::DiagnosticStatusWrapper &stat, bool & allOk);
  
  std::vector<std::string> getChildren(
    TiXmlElement* parentElement,std::string nodeName);

  std::vector<TiXmlDocument*> docsVector_;

  InterfacesXmlParser parser_;
  interface_tester::TFMonitor tfMonitor_;

};

#endif

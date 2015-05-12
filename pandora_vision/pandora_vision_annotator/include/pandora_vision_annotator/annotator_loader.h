/******************************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015 P.A.N.D.O.R.A. Team.
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

#ifndef PANDORA_VISION_ANNOTATOR_ANNOTATOR_LOADER_H
#define PANDORA_VISION_ANNOTATOR_ANNOTATOR_LOADER_H

#include "pandora_vision_annotator/annotator_tools.h"
#include "ui_pandora_annotator.h"
/**
@namespace pandora_vision
@brief The main namespace for pandora vision
**/ 
namespace pandora_vision
{
  /**
  @class CLoader
  @brief Implements the low level Qt functionalities of the main window. Inherits from Ui_MainWindow (generated from an ui file) and QMainWindow. 
  **/ 
  class CLoader : public Ui_MainWindow, public QMainWindow
  {
    //------------------------------------------------------------------------//
    private:
      //!< The number of input arguments
      int   argc_;
      //!< The input arguments
      char**  argv_;
      
      //!< True if the exit signal was emmited
      bool close_signal_;
      //!< The exit event (when occurs)
      QCloseEvent *event_;
      
    //------------------------------------------------------------------------//
    public:
      
      //!< The action of properties button
      //QAction *actionProperties;
         
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CLoader(int argc, char **argv);
      
      /**
      @brief Overloading of closeEvent function from QMainWindow
      @param event [QCloseEvent*] The exit event
      @return void
      **/
      void closeEvent(QCloseEvent *event);
      
      /**
      @brief Returns the exit event
      @return QEvent* 
      **/
      QEvent* getCloseEvent(void);
      
      /**
      @brief Returns true if a close event was triggered
      @return bool
      **/
      bool closeTriggered(void);
      
      /**
      @brief Shuts down the main window
      @return void
      **/
      void shutdown(void);
      
  };
  
}// namespace pandora_vision

#endif  // PANDORA_VISION_ANNOTATOR_ANNOTATOR_LOADER_H

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
* Author: Michael Skolarikis
*********************************************************************/

#ifndef TRACKERCHAIN_H
#define TRACKERCHAIN_H

#include <stdint.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

#include "thing.h"
#include "ros/ros.h"
namespace pandora_vision
{
  class TrackerChain : public std::vector<Thing*>
  {
    
    public:										
      
      TrackerChain();					// constructor
      ~TrackerChain();				// destructor
      
      void push(Thing* thingPtr);		// push a Thing* in the chain
          
      uintptr_t m_id; 				// id of chain
      
      bool active;					// true if the chain's last blob, is at most MIN_INACTIVITY frames old
      bool inactive; 					// true if the chain, is at least, is at least MAX_INACTIVITY frames old 
      int inactivity;					// shows how many frames have been elapsed, since the last blob has been inserted
      double probability;				// probability of the chain's last blob
      
      CvScalar color;					// color, with which the chain's blobs will be filled
  };
}
#endif

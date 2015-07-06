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
* Author:     Konstantinos Panayiotou   <klpanagi@gmail.com>
* Maintainer: Konstantinos Panayiotou   <klpanagi@gmail.com>
*********************************************************************/
#ifndef EPOS2_HANDLER_SERIAL_EPOS2_HANDLER_H
#define EPOS2_HANDLER_SERIAL_EPOS2_HANDLER_H

#include "epos_handler/abstract_epos_handler.h"
#include "epos2_gateway/epos2_gateway.h"

namespace pandora_hardware_interface
{
namespace motor
{

  struct Epos2Controller
  {
    uint16_t nodeId_;
    uint32_t errorCode_;
    std::string motorId_;
    uint16_t state_;
    int32_t rpm_;
    int16_t current_;
  };


  // ====================Serial Epos2 Handler Class========================

  class SerialEpos2Handler: public AbstractEposHandler
  {
    private:
      /*! NodeHandler under private namespace "~/epos2config"*/
      ros::NodeHandle epos2_nh_;
      boost::scoped_ptr<Epos2Gateway> epos2Gateway_;
      std::vector<Epos2Controller*> epos2Controllers_;
      Epos2Controller* rightFrontMotor_;
      Epos2Controller* rightRearMotor_;
      Epos2Controller* leftFrontMotor_;
      Epos2Controller* leftRearMotor_;
      uint16_t gatewayId_;

      /*
       * operation_mode_ = 0 ==> velocity mode
       * operation_mode_ = 1 ==> current mode
       */
      int operation_mode_;

    public:
      /*!
       * @brief Constructor 
       */
      SerialEpos2Handler(void);

      /*!
       * @brief Destructor
       */
      virtual ~SerialEpos2Handler();

      /*!
       * @brief Reads current velocity (rpm) from motor controllers
       * @param leftRearRpm Left-Rear wheel motor velocity in rpm
       * @param leftFrontRpm Left-Front wheel motor velocity in rpm
       * @param rightRearRpm Right-Rear wheel motor velocity in rpm
       * @param rightFrontRpm Right-Front wheel motor velocity in rpm
       * @return Void
       */
      virtual void getRPM(
        int* leftRearRpm,
        int* leftFrontRpm,
        int* rightRearRpm,
        int* rightFrontRpm);

      /*!
       * @brief Reads output current (mA) from motor controllers
       * @param leftRearCurrent Left-Rear wheel motor current in mA
       * @param leftFrontCurrent Left-Front wheel motor current in mA
       * @param rightRearCurrent Right-Rear wheel motor current in mA
       * @param rightFrontCurrent Right-Front wheel motor current in mA
       * @return Void
       */
      virtual void getCurrent(
        int* leftRearCurrent,
        int* leftFrontCurrent,
        int* rightRearCurrent,
        int* rightFrontCurrent);


      /*!
       * @TODO -- Doxy
       */
      virtual Error getError();


      /*!
       * @brief Writes velocity commands (rpm) to motor cotrollers
       * @param leftRpm   Left side velocity in rpm
       * @param rightRpm  Right side velocity in rpm
       * @return Void
       */
      virtual uint16_t writeRPM(const int leftRpm, const int rightRpm);


      /*!
       * @brief Reads motor controller states and stores the values
       *  in a private scope
       */
      void readStates(void);


      /*!
       * @TODO -- Doxy
       */
      void stateHandle(void);


      /*!
       * TODO -- Doxy
       */
      void getTorque(
        double* leftRearTorque,
        double* leftFrontTorque,
        double* rightRearTorque,
        double* rightFrontTorque);


      /*!
       * @brief Converts single WHEEL Torque to motor current
       */
      int16_t torqueToCurrent(
        double _input_torque);


      /*!
       * @brief Converts single motor current to WHEEL Torque 
       */
      double currentToTorque(
        int _input_current);


      /*!
       * @brief Writes torque commands to motor controllers 
       * (needs to convert torques to currents as well)
       */
      uint16_t writeTorques(
        double leftRearTorque,
        double leftFrontTorque,
        double rightRearTorque,
        double rightFrontTorque);


      /*!
       * @brief Switches between current and velocity mode 
       *        Constuctor defaul = velocity mode 
       * 
       * @param mode  mode = 0 => velocity mode
       *              mode = 1 => current mode
       */
      void setMode(int mode);


      /*!
       * @TODO -- Doxy
       */
      int getMode(void);
  };
}  // namespace motor
}  // namespace pandora_hardware_interface
#endif  // EPOS2_HANDLER_SERIAL_EPOS2_HANDLER_H

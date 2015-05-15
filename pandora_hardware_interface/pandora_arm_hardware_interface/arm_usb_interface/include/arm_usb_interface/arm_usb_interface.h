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
 * Author: George Kouros
 * Author: Nikos Taras
 * Author: Orestis Zachariadis
 *********************************************************************/

#ifndef ARM_USB_INTERFACE_ARM_USB_INTERFACE_H
#define ARM_USB_INTERFACE_ARM_USB_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>

#include <ros/ros.h>

namespace pandora_hardware_interface
{
namespace arm
{

#define NACK 0xFFF1
#define ACK 0x0001


#define COMMAND_ENCODER 1         //!< encoder ead command code
#define COMMAND_SONAR_LEFT 2      //!< Left Sonar read command code
#define COMMAND_SONAR_RIGHT 3     //!< Right Sonar read command code
#define COMMAND_CO2 4             //!< CO2 read command code
#define COMMAND_BATTERY_MOTOR 5   //!< Motor Battery read command code
#define COMMAND_BATTERY_SUPPLY 6  //!< Supply Battery read command code
#define COMMAND_GEYE_CENTER 7     //!< Center Grideye read command
#define COMMAND_GEYE_LEFT 8       //!< Left Grideye read command
#define COMMAND_GEYE_RIGHT 9      //!< Right Grideye read command

#define NO_ERROR 1                 //!< no-error code
#define WRITE_ERROR -1             //!< write error code
#define READ_ERROR -2              //!< read error code
#define INCORRECT_NUM_OF_BYTES -3  //!< incorrect number of bytes error code
#define RECEIVED_NACK -4
#define SELECT_ERROR -5
#define READ_TIMEOUT -6

#define CO2_NBYTES 4         //!< Number of bytes of incoming CO2 data
#define SONAR_NBYTES 2       //!< Number of bytes of incoming Sonar data
#define ENCODER_NBYTES 2     //!< Number of bytes of incoming Encoder data
#define BATTERY_NBYTES 2     //!< Number of bytes of incoming Battery data
#define COMMAND_NBYTES 1     //!< Number of bytes of outgoing command
#define GEYE_NBYTES 64       //!< Number of bytes of incoming GridEYE data
#define NACK_NBYTES 2

/**
* @class ArmUSBInterface
* @brief Communicates with an arm uC board to get sensor measurements
**/
class ArmUsbInterface : private boost::noncopyable
{
 public:
  /**
   * @brief Default constructor
  **/
  ArmUsbInterface();

  /**
   * @brief Default Destructor
   * @details Closes USB port
  **/
  ~ArmUsbInterface();

  /**
   * @brief Opens USB port
  **/
  void openUsbPort();

  int readData(int fd, uint8_t bufOut, uint8_t command_bytes, uint8_t* readBuf);

  /**
   * @brief Reads 8x8 thermal image of the selected grideye
   * @attention If the uController detects a malfunction in a sensor it returns
   * zeros in place of its readings.
   * @note Although the uController can return the most recent reading whenever
   * it is asked, there is no point asking for data much more frequently than
   * two times the sensor speed. This sensor's frequency is 10 Hz.
   * @param grideyeSelect [const char&] : 'C' for Center GridEYE, 'L' for left,
   *  'R' for right
   * @param values [uint8_t*] : pointer to first byte of result
   * @return int : 1 for a successful read, -1 for write error, -2 for read 
   * error,-3 for incorrect number of bytes read
  **/
  int readGrideyeValues(const char& grideyeSelect, uint8_t* values);

  /**
   * @brief Reads a distance measurement from the selected sonar sensor
   * @param sonarSelect [const char&] : used to select which sensor
   * measurement to read
   * @param values [uint16_t*] : pointer to result
   * @return int : 1 for a successful read, -1 for write error, -2 for
   * read error, -3 for incorrect number of bytes read
   * @attention If the uController detects a malfunction in a sensor it returns
   * zeros in place of its readings.
   * @note Although the uController can return the most recent reading whenever
   * it is asked, there is no point asking for data much more frequently than
   * two times the sensor speed. This sensor's frequency is 10 Hz.
  **/
  int readSonarValues(const char& sonarSelect, uint16_t* value);

  /**
   * @attention If the uController detects a malfunction in a sensor it returns
   * zeros in place of its readings.
   * @note Although the uController can return the most recent reading whenever
   * it is asked, there is no point asking for data much more frequently than
   * two times the sensor speed. This sensor's frequency is 2 Hz.
   * @param values [float*] : pointer to result
   * @return int: 1 for successfull read, -1 for write error, -2 for read error,
   * -3 for incorrect number of bytes read
  **/
  int readCo2Value(float* value);

  /**
   * @brief Reads a measurement from the encoder
   * @param value [uint16_t] : pointer to result
   * @return int : 1 if read was successful, -1 for write error, -2 for read 
   * error, -3 for incorrect number of bytes read
   * @attention If the uController detects a malfunction in a sensor it returns
   * zeros in place of its readings.
   * @note Although the uController can return the most recent reading whenever
   * it is asked there is no point asking for data much more frequently than
   * two times the sensor speed.
  **/
  int readEncoderValue(uint16_t* value);

  /**
   * @brief Reads a voltage measurement from the selected battery
   * @param batterySelect [const char&] : used to select which battery's
   * voltage to read
   * @param values [uint16_t*] : pointer to result
   * @return int : 1 if read was successful,
   * -1 for write error, -2 for read error,
   * -3 for incorrect number of bytes read
   * @note Although the uController can return the most recent reading whenever
   * it is asked there is no point asking for data much more frequently than
   * two times the sensor speed.
  **/
  int readBatteryValues(const char& batterySelect, uint16_t* value);

 private:
  /**
   * @brief Attempts to reconnect to the USB port in case of a disconnection
   * @details inserts a 1.5 sec delay between the closing of the connection to
   * the usb port and then attempts to reopen it.
   * @return void
  **/
  void reconnectUsb();
  int fd;  ///< File Descriptor
};

}  // namespace arm
}  // namespace pandora_hardware_interface

#endif  // ARM_USB_INTERFACE_ARM_USB_INTERFACE_H

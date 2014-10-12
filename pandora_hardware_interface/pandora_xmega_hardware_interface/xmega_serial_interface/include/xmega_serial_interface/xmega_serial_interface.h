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
* Author: Michael Niarchos
* Author: Chris Zalidis
*********************************************************************/

/** @file xmega_serial_interface.h
 *  @brief Serial interface drivers header file for xMega uController.
 *
 *  This contains classes and prototypes for xMega Com driver
 *  and eventually any definitions and globals.
 *
 *  @author Michael Niarchos
 *  @author Chris Zalidis
 *  @author Konstantinos Panayiotou
 *  @bug No known bugs.
 */



#ifndef XMEGA_SERIAL_INTERFACE_XMEGA_SERIAL_INTERFACE_H
#define XMEGA_SERIAL_INTERFACE_XMEGA_SERIAL_INTERFACE_H

#include <stdexcept>
#include <sys/time.h>
#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>

#include <ros/ros.h>
#include <serial/serial.h>

#include "xmega_serial_interface/enums.h"
#include "xmega_serial_interface/default_sensor.h"
#include "xmega_serial_interface/range_sensor.h"
#include "xmega_serial_interface/battery_sensor.h"
#include "xmega_serial_interface/encoder_sensor.h"

#define DEVICE "/dev/ttyS0"
#define SPEED       115200
#define TIMEOUT     100

namespace pandora_hardware_interface
{
namespace xmega
{
/*!
 * @class SerialIO
 * @brief Class used for serial communication (read/write) with xMega.
 */
class SerialIO : private boost::noncopyable
{
 public:
  /*!
   *  Default constructor.
   *  @param device Device Com Port under Unix OS.
   *  @param speed Device Com speed (baudrate).
   *  @param timeout Timeout value.
   */
  SerialIO(const std::string& device,
           int speed,
           int timeout);

  /*!
   * @brief Opens Communication port.
   *  @return Void.
   */
  void openDevice();

  /*!
   * @brief Reads two (2) bytes from port buffer and checks if they match the "Start of a new data package" char sequence.
   * Char sequence: FF(Form Feed) + LF(Line Feed) = (0x0C) + (0x0A)
   * @return Next state (Idle or Sucess on new package(Read Size)).
   */
  int readMessageType();

  /*!
   * @brief Reads data package size.
   *
   * Reads 5 bytes. The first four bytes indicates the data package size.
   * Last byte is the LF byte ('\n').
   *
   * @param dataSize Data package size value.
   * @return Next state (Idle or Read Data).
   */
  int readSize(uint16_t *dataSize);

  /*!
   * @brief Reads actual data.
   *
   * Reads and fills he data buffer. Data buffer only includes sensor measurements data.
   *
   *  @param dataSize Size of data.Includes crc data.
   *  @param dataBuffer Data buffer.
   *  @return Next state Read_CRC or ...?(missing)
   */
  int readData(uint16_t dataSize, unsigned char *dataBuffer);

  /*!
   * @brief Reads CRC.
   *
   * Reads CRC from xMega and checks if it matches the calculated crc.
   *
   * @return Next State (Ack / Nak).
   */
  int readCRC();

  /*!
   * @brief Writes data to buffer.
   * @param data output data buffer.
   * @param size Number of bytes to write from data buffer.
   * @return True if Success. False if failed.'
   */
  bool write(const uint8_t *data, size_t size);

  /*!
   * @brief Terminates device communication.
   *
   * Terminates communication. Flushes I/O buffers before termination and closes serial com port.
   *
   * @return Void.
   */
  void closeDevice();

  /*! < Default destructor. */
  ~SerialIO();

 private:
  /*! < Calculated CRC value for every data package received. */
  int CRC_;
  /*! < Device com port location (e.g. "/dev/ttyS0"). */
  const std::string device_;
  /*! < Serial communication speed (baudrate). */
  const int speed_;
  /*! < Timeout value. */
  const int timeout_;
  boost::scoped_ptr<serial::Serial> serialPtr_;
};

class XmegaSerialInterface : private boost::noncopyable
{
 public:
  /*! < Default constuctor. */
  XmegaSerialInterface();

  /*! < Default destructor. */
  ~XmegaSerialInterface();

  /*!
   * @brief Call to open device serial com port.
   *
   * @return Void.
   */
  void init();

  /*!
   * @brief Call to read a data package.
   * @return Void.
   */
  void read();

  /*!
   * @brief Get battery voltage level measurements method.
   * @param [in] psuVoltage Voltage level measurement of electonics battery.
   * @param [in] psuVoltage Voltage level measurement of motors battery.
   * @return Void
   */
  inline void getBatteryData(double* psuVoltage, double* motorVoltage) const
  {
    *psuVoltage = batterySensor_.psuVoltage;
    *motorVoltage = batterySensor_.motorVoltage;
  }

  /*!
   * @brief Get Sonars range map method.
   * @return RangeMap
   */
  inline RangeMap getRangeData() const
  {
    return rangeSensors_.sensors;
  }

  /*!
   * @brief Get Rotary encoder degrees measurement from differential shaft.
   * @return Double.
   */
  inline double getEncoderDegrees() const
  {
    return encoderSensor_.degrees;
  }

 private:
  /*!
   * @brief Read and handles a data package.
   * @return Void.
   */
  void receiveData();

  /*!
   * @brief Processes sensor measurements data.
   * @return Success / Error.
   */
  int processData();

  /*!
   * @Returns a SensorBase object pointer.
   * @param [in] sensorType Sensor Type (battery/sonar/encoder...)
   * @return SensorBase object.
   */
  SensorBase* getSensor(int sensorType);

  /*! < Global data buffer pointer. */
  unsigned char *pdataBuffer_;

  /*! < Current communication state. */
  int currentState_;

  timeval tim_;
  double t1_;
  double t2_;
  uint16_t dataSize_;

  DefaultSensor defaultSensor_;
  /*! < Battery sensor object*/
  BatterySensor batterySensor_;
  /*! < Encoder sensor object*/
  EncoderSensor encoderSensor_;
  /*! < Sonars sensor object*/
  RangeSensor rangeSensors_;

  SerialIO* serialIO_;
};

static unsigned char myatoi(char *array, int size);

}  // namespace xmega
}  // namespace pandora_hardware_interface

#endif  // XMEGA_SERIAL_INTERFACE_XMEGA_SERIAL_INTERFACE_H

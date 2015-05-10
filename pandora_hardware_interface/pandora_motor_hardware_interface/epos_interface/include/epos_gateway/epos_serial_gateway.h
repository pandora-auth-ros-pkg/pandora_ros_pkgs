/***************************************************************************
 *   Copyright (C) 2008-2011 by Charalampos Serenis <info@devel.serenis.gr>*
 *   Author: Charalampos Serenis <info@devel.serenis.gr>                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,  *
 *   MA 02110-1301, USA.                                                   *
 ***************************************************************************/


/**
 * \file EposRs232Gateway.h
 * \brief RS232 interface for a EPOS device gateway
 *
 * Defines the implementation for the RS232 device gateway. This class
 * EposRs232Gateway inherits the AbstractEposGateway.
 *
 * EposRs232Gateway.h declares the following methods:
 * \li \c int getSoftwareVersion(void);
 * \li \c epos::status EposRs232Gateway::sendFrame(
 * 	unsigned char opCode,
 * 	epos::Word *data,
 * 	unsigned short length,
 * 	epos::Word *response);
 * \li \c epos::status EposRs232Gateway::readObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	epos::DWord *responce);
 * \li \c epos::status EposRs232Gateway::readObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	epos::Word *responce);
 * \li \c epos::status EposRs232Gateway::readObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	char *responce);
 * \li \c epos::status EposRs232Gateway::writeObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	epos::DWord data);
 * \li \c epos::status EposRs232Gateway::writeObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	epos::Word data);
 * \li \c epos::status EposRs232Gateway::writeObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	char data);
 *
 * \author Charalampos Serenis
 * \author Electical and Computer Engineer
 * \author Department of Electrical and Computer Engineering
 * \author Aristotle University of Thessaloniki, Greece
 *
 */

#ifndef EPOS_GATEWAY_EPOS_SERIAL_GATEWAY_H
#define EPOS_GATEWAY_EPOS_SERIAL_GATEWAY_H

#include <string>
#include <pthread.h>
#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>

#include <serial/serial.h>
#include "epos_gateway/abstract_epos_gateway.h"

namespace pandora_hardware_interface
{
namespace motor
{

/**
 * \brief RS232 EPOS gateway
 *
 * EposRs232Gateway provides access to EPOS devices connected to the PC through
 * an RS232 port. Class provides generic functions for read and write commands
 * through the gateway
 *
 * \author Charalampos Serenis
 * \author Electical and Computer Engineer
 * \author Department of Electrical and Computer Engineering
 * \author Aristotle University of Thessaloniki, Greece
 *
 */

class EposSerialGateway : public AbstractEposGateway, private boost::noncopyable
{
 public:
  // Construct an EposRs232Gateway object and open EPOS device for read/write
  /**
   *
   * \param[in] device the device name of the serial port the EPOS device is
   * connected. e.g. "/dev/ttyS0"
   * \param[in] baudRate the baudrate that should be used for communication.
   * valid values are:
   * \li 115200
   * \li 57600
   * \li 38400
   * \li 19200
   * \li 9600
   * \li 4800
   * \li 2400
   * \li 1800
   * \li 1200
   * \li 600
   * \li 300
   * \li 200
   * \li 150
   * \li 134
   * \li 110
   * \li 75
   * \li 50
   * \param[in] timeout timeout in ms. This is used when attempting to read data
   * from the serial port
   *
   */
  EposSerialGateway(
      const std::string& device,
      unsigned int baudRate = 38400,
      unsigned int timeout = 500);

  /** Send a CAN frame using the EPOS gateway
   *
   * this function can be used to send new, or custom CAN commands to the
   * CANOpen bus.
   *
   * \param[in] opCode the operation code of the command
   * \param[in[ data the data that we wish to write to the CANOpen bus
   * \param[in] length the size of the data array
   * \param[in] response the data received from the device as a response to the
   * command
   */
  epos::CommandStatus sendFrame(
      unsigned char opCode,
      epos::Word* data,
      uint8_t length,
      epos::Word* response);

  /** Read object dictionary entry (4 Data Bytes and less)
  *
  * Read an object value at the given Index and SubIndex from the
  * Object Dictionary.
  *
  * \param[in] nodeId the node ID of the device from which to read
  * the data
  * \param[in] index the index of the Object Dictionary we wish to
  * read
  * \param[in] subIndex the sub-index of the Dictionary we wish to
  * read
  * \param[out] responce the data read
  * \return the status of the operation
  */
  epos::CommandStatus readObject(
      unsigned char nodeID,
      uint16_t index,
      unsigned char subIndex,
      epos::DWord* responce);
  epos::CommandStatus readObject(
      unsigned char nodeID,
      uint16_t index,
      unsigned char subIndex,
      epos::Word* responce);
  epos::CommandStatus readObject(
      unsigned char nodeID,
      uint16_t index,
      unsigned char subIndex,
      char* responce);

  /** Write Object Dictionary Entry (4 Data Bytes and less)
  *
  * Write an object value to the given Index and SubIndex from the
  * Object Dictionary.
  *
  * \param[in] nodeID the node ID of the device from which to read the
  * data
  * \param[in] index the index of the Object Dictionary we wish to
  * read
  * \param[in] subIndex the sub-index of the Dictionary we wish to
  * read
  * \param[in] data the data that should be written
  * \return the status of the operation
  */
  epos::CommandStatus writeObject(
      unsigned char nodeID,
      uint16_t index,
      unsigned char subIndex,
      epos::DWord data);
  epos::CommandStatus writeObject(
      unsigned char nodeID,
      uint16_t index,
      unsigned char subIndex,
      epos::Word data);
  epos::CommandStatus writeObject(
      unsigned char nodeID,
      uint16_t index,
      unsigned char subIndex,
      char data);

private:
    // Rs232 object used to access underlaying hardware
    boost::scoped_ptr<serial::Serial> serialPtr_;
    /** POSIX mutex to lock when a transmition is ongoing. Since the RS232 EPOS
     * protocol requires a handshake, transmitions are not unary operations. */
    pthread_mutex_t gatewayMutex;
    /** function that implements the CRC16 CCITT hashing algorithm required by
     * EPOS RS232 protocol */
    epos::Word crc16CCITT(epos::Word* data, unsigned int length);
    /// variable to store object initialization status
    bool initialized;
    /// receive an ACK response from the EPOS controller
    epos::CommandStatus getACK(void);
    /// send an ACK response to the EPOS controller
    epos::CommandStatus ACK(void);
};

}  // namespace motor
}  // namespace pandora_hardware_interface
#endif  // EPOS_GATEWAY_EPOS_SERIAL_GATEWAY_H

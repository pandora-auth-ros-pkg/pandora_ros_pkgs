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
#ifndef EPOS_GATEWAY_ABSTRACT_EPOS_GATEWAY_H
#define EPOS_GATEWAY_ABSTRACT_EPOS_GATEWAY_H

#include <stdint.h>

/**
 * \file AbstractEposGateway.h
 * \brief Abstract interface for a EPOS / EPOS P device gateway
 *
 * Declares all the neccessary datatypes and interfaces to access an EPOS /
 * EPOS P motor controller. This interface is provided in order to support
 * different hardware implementations of the gateway, such as RS232 interface
 * USB interface, etc.
 * AbstractEposGateway.h declares the following namespaces:
 * \li \c epos
 *
 * AbstractEposGateway.h declares the following typedefs:
 * \li \c Word
 * \li \c DWord
 * \li \c Byte
 *
 * AbstractEposGateway.h declares the following enumerations:
 * \li \c CommandStatus
 *
 * AbstractEposGateway.h declares the following abstract classes:
 * \li \c AbstractEposGateway
 *
 * AbstractEposGateway.h declares the following methods:
 * \li \c int getSoftwareVersion(void);
 * \li \c epos::status AbstractEposGateway::sendFrame(
 * 	unsigned char opCode,
 * 	epos::Word *data,
 * 	unsigned short length,
 * 	epos::Word *response);
 * \li \c epos::status AbstractEposGateway::readObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	epos::DWord *responce);
 * \li \c epos::status AbstractEposGateway::readObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	epos::Word *responce);
 * \li \c epos::status AbstractEposGateway::readObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	char *responce);
 * \li \c epos::status AbstractEposGateway::writeObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	epos::DWord data);
 * \li \c epos::status AbstractEposGateway::writeObject(
 * 	unsigned char nodeID,
 * 	uint16_t index,
 * 	unsigned char subIndex,
 * 	epos::Word data);
 * \li \c epos::status AbstractEposGateway::writeObject(
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

namespace pandora_hardware_interface
{
namespace motor
{
namespace epos
{
  /// EPOS word (16bit)
  typedef uint16_t Word;
  /// Double (long) EPOS word (32bit)
  typedef uint32_t DWord;
  /// Short EPOS word (8bit)
  typedef unsigned char Byte;

  /**
   * \brief enumeration of error codes during device communication
   *
   * For communication with the device, a protocol is implemented. In the
   * event that the communication was successful, each protocol command
   * returns CommandStatus::SUCCESS.
   *
   */
  enum CommandStatus
  {
    /**
     * \brief Command sent successfully
     *
     * This status code is returned by the implementation in the event
     * of a successful transaction with the device. Please, note that
     * this doesn't mean the command execution was successful, only the
     * data transfer was. In some cases the device my return error codes
     * using the response data, signifying the result of the command
     * execution. In the event that a transaction does not return SUCCESS
     * the returned data are undefined.
     *
     */
    SUCCESS = 0,
    /**
     * \brief Device busy
     *
     * This error code is returned if the device signals that it is not
     * ready to receive new commands.
     *
     */
    BUSY,
    ///Device replied with NACK
    NACK,
    ///Time out
    TIMEOUT,
    ///RS232 port error. Usually port disconnected
    RS232,
    ///Invalid API usage. Read documentation
    API,
    ///Unexpected responce during handshake. Please disconnect and
    ///reconnect to resyncronize gateway
    RESYNC,
    ///Responce from device violates protocol. Check for changes in
    ///updated firmware
    PROTOCOL,
    DEFAULT
  };

}  // namespace epos
  /**
   * \brief Abstract gateway for EPOS devices
   *
   * All gateways for EPOS (P) devices must implement this class to
   * provide compatibility with new devices and hardware implementations.
   *
   * \author Charalampos Serenis
   * \author Electical and Computer Engineer
   * \author Department of Electrical and Computer Engineering
   * \author Aristotle University of Thessaloniki, Greece
   *
   */


  class AbstractEposGateway
  {
    public:
      /** Send a custom frame using the EPOS gateway
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
      virtual epos::CommandStatus sendFrame(
        unsigned char opCode,
        epos::Word* data,
        uint8_t length,
        epos::Word* response)=0;

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
      virtual epos::CommandStatus readObject(
        unsigned char nodeId,
        uint16_t index,
        unsigned char subIndex,
        epos::DWord* responce)=0;
      virtual epos::CommandStatus readObject(
        unsigned char nodeId,
        uint16_t index,
        unsigned char subIndex,
        epos::Word* responce)=0;
      virtual epos::CommandStatus readObject(
        unsigned char nodeId,
        uint16_t index,
        unsigned char subIndex,
        char* responce)=0;

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
      virtual epos::CommandStatus writeObject(
        unsigned char nodeId,
        uint16_t index,
        unsigned char subIndex,
        epos::DWord data)=0;
      virtual epos::CommandStatus writeObject(
        unsigned char nodeId,
        uint16_t index,
        unsigned char subIndex,
        epos::Word data)=0;
      virtual epos::CommandStatus writeObject(
        unsigned char nodeId,
        uint16_t index,
        unsigned char subIndex,
        char data)=0;
  };
}  // namespace motor
}  // namespace pandora_hardware_interface

#endif  // EPOS_GATEWAY_ABSTRACT_EPOS_GATEWAY_H

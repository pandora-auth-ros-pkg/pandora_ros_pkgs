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

#include <iostream>
#include "epos_gateway/epos_serial_gateway.h"

namespace pandora_hardware_interface
{
namespace motor
{

EposSerialGateway::EposSerialGateway(
  const std::string& device, unsigned int baudRate, unsigned int timeout)
{
  // gateway is not initialized until we actually connect to the RS232 port
  initialized = false;
  // initialize POSIX thread mutex, for use during read/write operations
  pthread_mutex_init(&gatewayMutex, NULL);

  try
  {
    serialPtr_.reset(
      new serial::Serial(
        device,
        baudRate,
        serial::Timeout::simpleTimeout(baudRate)));
  }
  catch (serial::IOException& ex)
  {
    // failed to connect to serial port
    std::cerr << "error: EposRs232Gateway: " << ex.what() << std::endl;
    return;
  }

  // we are connected, set object as initialized
  initialized = true;
}

epos::CommandStatus EposSerialGateway::getACK(void)
{
  uint8_t c;

  // read a single character from the RS232 port
  size_t n = serialPtr_->read(&c, 1);

  // check the number of characters that were actually read
  if (n == 0)  // no characters read, timeout
    return epos::TIMEOUT;
  else if (n == 1 && c == 'F')  // one character read, it's a NACK
    return epos::NACK;
  else if (n == 1 && c != 'O')  // Should never happen, violates EPOS Communication Guide
    return epos::RESYNC;
  else if (n != 1)  // RS232 error
    return epos::RS232;

  // 1 character read, equal to 'O' (ACK)
  return epos::SUCCESS;
}

epos::CommandStatus EposSerialGateway::ACK(void)
{
  // ACK signal is a single character equal to 'O'
  size_t n = serialPtr_->write("O");

  // Rs232 write method should return 0, if all went well
  if (n != 1) return epos::RS232;

  // Allright
  return epos::SUCCESS;
}

epos::CommandStatus EposSerialGateway::sendFrame(
  unsigned char opCode, epos::Word *data, uint8_t length,
    epos::Word *response)
{
  // In order to debug the actual communication between the PC and the RS232 port
  // you can define the DEBUG_EposRs232Gateway macro. This will enable debugging
  // code that prints the communication to the standard output. This generates
  // allot of output, and should be used only for debugging purposes. Having the
  // printout of the communication is sometimes the only to way to debug since
  // underlying synchronization issues exist (i.e. timeouts)

  // #define DEBUG_EposRs232Gateway

  // check to see if gateway is initialized
  if (!initialized)
  {
    // cannot use the gateway if the object is not initialized first
    std::cerr << "error: EposRs232Gateway: sendFrame: cannot send frame, " <<
      "EPOS is not connected. Please use EposRs232Gateway::connect" << std::endl;
    return epos::API;
  }

  // frame size must be a positive integer no greater than 255. This restriction
  // is enforced by hardware, and protocol specifications
  if (length > 255 || !length)
  {
    std::cout << "error: EposRs232Gateway: sendFrame: invalid frame data size: " << length << std::endl;
    return epos::API;
  }

  // *** input arguments are valid *** //
  // *** Calculate CRC *** //

  // CRC is calculated on the transmition data + the op code and the data length
  // The op code and the data length are stuffed in a single EPOS word
  epos::Word crcData[length + 1];

  // load the first word of the data input for the CRC method, opcode + length
  crcData[0] = (opCode << 8) | (length - 1);

  // EPOS CRC calculation requires data to be Big-Endian (MSB first)
  for (unsigned int i = 1; i < length + 1; ++i)
  {
    crcData[i] = ((data[i - 1] / 256) << 8) | (data[i - 1] % 256);
  }
  // get CRC
  epos::Word crc = crc16CCITT(&crcData[0], length + 1);

  // Load the transmition buffer
  // Frame data are sent LSB first
  uint8_t outData[2 * length];
  for (unsigned int i = 0; i < length; ++i)
  {
    outData[2 * i] = data[i] % 256;
    outData[2 * i + 1] = data[i] / 256;
  }

  // Communication variables
  uint8_t c;
  size_t n;
  uint8_t buffer[128];
  epos::CommandStatus state;

  // We are about to start transmiting data, lock the communication mutex
  pthread_mutex_lock(&gatewayMutex);

#ifdef DEBUG_EposRs232Gateway
  printf("Op Code-> 0x%02X\n", opCode & 0xFF);
#endif

  //  === State 1: Send opCode ===
  if (serialPtr_->write(&opCode, 1) != 1)
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: unexpected RS232 " <<
      "error while attempting to send op code" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RS232;
  }
  //  === State 2: Wait for ACK ===
  state = getACK();
  if (state == epos::NACK)
  {
    std::cerr << "warning: EposRs232Gateway: sendFrame: gateway busy" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return epos::BUSY;
  }
  if (state != epos::SUCCESS)
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: unexpected error while attempting to send op code" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return state;
  }
#ifdef DEBUG_EposRs232Gateway
  printf(" <-ACK\n");
#endif

#ifdef DEBUG_EposRs232Gateway
  printf("Frame Size-> 0x%02X\n", (length - 1) & 0xFF);
#endif
  // === State 3: Send len-1 ===
  uint8_t temp = length - 1;
  if (serialPtr_->write(&temp, 1) != 1)
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: unexpected RS232 " <<
      "error while attempting to send data length" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RS232;
  }
#ifdef DEBUG_EposRs232Gateway
  printf("Data-> ");
  for (int temp = 0; temp < ((length << 1) - 1); ++temp)
  {
    printf("0x%02X, ", outData[temp] & 0xFF);
  }
  printf("0x%02X\n", outData[((length << 1) - 1)] & 0xFF);
#endif
  // === State 4: Send data ===
  n = serialPtr_->write(outData, 2 * length);
  if (n != (2 * length))
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: RS232 didn't finish " <<
      "writing data frame" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RS232;
  }

  // === State 5: Send CRC ===
  buffer[0] = crc % 256;
  buffer[1] = crc / 256;
#ifdef DEBUG_EposRs232Gateway
  printf("CRC-> 0x%02X, 0x%02X\n", buffer[0] & 0xFF, buffer[1] & 0xFF);
#endif
  n = serialPtr_->write(buffer, 2);
  if (n != 2)
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: unexpected RS232 " <<
      "error while attempting to send CRC16" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RS232;
  }
  // === State 6: Wait for ACK ===
  state = getACK();
  if (state != epos::SUCCESS)
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: unexpected error while waiting for end ACK" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return state;
  }
#ifdef DEBUG_EposRs232Gateway
  printf(" <-ACK\n", buffer[0] & 0xFF, buffer[1] & 0xFF);
#endif
  // Command Sent Successfully

  //  === State 7: Wait for responce opCode ===
  n = serialPtr_->read(&c, 1);
  if (n == 0)
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: responce from gateway " <<
      "timed-out" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return epos::TIMEOUT;
  }
  else if (n == 1 && c != 0x00)
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: unexpected responce " <<
      "from gateway. Resyncronization needed." << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RESYNC;
  }
  else if (n != 1)
  {
    std::cerr << "error: EposRs232Gateway: sendFrame: unexpected RS232 " <<
      "error while waiting for responce opCode from gateway" << std::endl;
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RS232;
  }
#ifdef DEBUG_EposRs232Gateway
  printf(" 0x%02X <-OpCode\n", c & 0xFF);
#endif

  // === State 8: Send ACK ===
  state = ACK();
  if (state != epos::SUCCESS)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return state;
  }
#ifdef DEBUG_EposRs232Gateway
  printf("ACK-> \n", c & 0xFF);
#endif
  // === State 9: Receive len-1 ===
  n = serialPtr_->read(&c, 1);
  if (n == 0)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::TIMEOUT;
  }
  if (n != 1)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RS232;
  }
#ifdef DEBUG_EposRs232Gateway
  printf(" %02X <- Frame Size\n", c & 0xFF);
#endif
  int size = ((unsigned int)(c)) + 1;
  if (size > 63)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::PROTOCOL;
  }
  // === State 10: Receive data ===
  size <<= 1;
  uint8_t inData[256];
  n = serialPtr_->read(&inData[0], size);
  if (n == 0)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::TIMEOUT;
  }
#ifdef DEBUG_EposRs232Gateway
  printf(" ");
  for (unsigned int i = 0; i < (size - 1); ++i
  {
    printf("%02X, ", inData[i] & 0xFF);
  }
  printf("%02X <- Data\n", inData[size - 1] & 0xFF);
#endif
  if (n != size)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RESYNC;
  }

  // === State 11: Receive CRC ===
  n = serialPtr_->read(&c, 1);
  if (n == 0)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::TIMEOUT;
  }
  if (n != 1)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RS232;
  }

#ifdef DEBUG_EposRs232Gateway
  printf(" %02X, ");
#endif



  n = serialPtr_->read(&c, 1);
  if (n == 0)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::TIMEOUT;
  }
  if (n != 1)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return epos::RS232;
  }

#ifdef DEBUG_EposRs232Gateway
  printf("%02X <- CRC\n", buffer[0] & 0xFF, buffer[1] & 0xFF);
#endif

  // === State 12: Send ACK or NACK ===
  state = ACK();
  if (state == epos::SUCCESS)
  {
#ifdef DEBUG_EposRs232Gateway
    printf("ACK-> \n");
#endif
  }
  for (unsigned int i = 0; i < (size / 4); ++i)
  {
    uint16_t MSB, LSB;
    LSB = ((inData[4 * i + 1] & 0xFF) << 8) | (inData[4 * i] & 0xFF);
    MSB = ((inData[4 * i + 3] & 0xFF) << 8) | (inData[4 * i + 2] & 0xFF);
    response[i] = (MSB << 16) | LSB;
  }
  if (state != epos::SUCCESS)
  {
    pthread_mutex_unlock(&gatewayMutex);
    return state;
  }
  pthread_mutex_unlock(&gatewayMutex);
  return epos::SUCCESS;

#undef DEBUG_EposRs232Gateway
}

epos::CommandStatus EposSerialGateway::readObject(
  unsigned char nodeID,
  uint16_t index,
  unsigned char subIndex,
  epos::DWord *responce)
{
  return epos::DEFAULT;
}

epos::CommandStatus EposSerialGateway::readObject(
  unsigned char nodeID,
  uint16_t index,
  unsigned char subIndex,
  char *responce)
{
  return epos::DEFAULT;
}

epos::CommandStatus EposSerialGateway::readObject(
  unsigned char nodeID, uint16_t index, unsigned char subIndex,
  epos::Word *responce)
{
  epos::Word data[2];
  data[0] = index;
  data[1] = (nodeID << 8) | subIndex;
  return sendFrame(0x10, &data[0], 2, responce);
}

epos::CommandStatus EposSerialGateway::writeObject(
  unsigned char nodeID, uint16_t index, unsigned char subIndex,
  epos::DWord data)
{
  epos::Word internalData[4];
  epos::Word dataHigh, dataLow;
  uint32_t rawData = (uint32_t)data;
  dataHigh = rawData / 65536;
  dataLow = rawData % 65536;

  internalData[0] = index;
  internalData[1] = (nodeID << 8) | subIndex;
  internalData[2] = dataLow;
  internalData[3] = dataHigh;
  epos::Word ret[2];
  return sendFrame(0x11, internalData, 4, &ret[0]);
}

epos::CommandStatus EposSerialGateway::writeObject(
  unsigned char nodeID, uint16_t index, unsigned char subIndex,
  epos::Word data)
{
  epos::Word internalData[4];
  internalData[0] = index;
  internalData[1] = (nodeID << 8) | subIndex;
  internalData[2] = data;
  internalData[3] = 0;
  epos::Word ret[2];
  return sendFrame(0x11, internalData, 4, &ret[0]);
}

epos::CommandStatus EposSerialGateway::writeObject(
  unsigned char nodeID, uint16_t index, unsigned char subIndex, char data)
{
  epos::Word internalData[4];
  epos::Word wordData = 0;
  wordData |= data;
  internalData[0] = index;
  internalData[1] = (nodeID << 8) | subIndex;
  internalData[2] = wordData;
  internalData[3] = 0;
  epos::Word ret[2];
  return sendFrame(0x11, internalData, 4, &ret[0]);
}


epos::Word EposSerialGateway::crc16CCITT(epos::Word* data, unsigned int length)
{
  // Append a zero-word to packet
  uint16_t *packet, *temp;
  temp = packet = new uint16_t[length + 1];
  memcpy(packet, data, length * 2);
  packet[length] = 0;
  ++length;
  // Calculate CRC using polyonym: x^16+x^12+x^5+x^0
  uint16_t shifter, c;
  uint16_t carry;
  uint16_t CRC = 0;
  while (length--)
  {
    shifter = 0x8000;                 // Initialize BitX to Bit15
    c = *packet++;                // Copy next Data uint16_t to c
    do
    {
      carry = CRC & 0x8000;  // Check if Bit15 of CRC is set
      CRC <<= 1;             // CRC = CRC * 2
      if (c & shifter) ++CRC;  // CRC = CRC + 1, if BitX is set in c
      if (carry) CRC ^= 0x1021;  // CRC = CRC XOR G(x), if carry is true
      shifter >>= 1;         // Set BitX to next lower Bit, shifter = shifter/2
    }
    while (shifter);
  }
  delete[] temp;
  epos::Word ret;
  memcpy(&ret, &CRC, 2);
  return ret;
}

}  // namespace motor
}  // namespace pandora_hardware_interface

#ifndef JRK_INTERFACE_JRKSERIAL_H
#define JRK_INTERFACE_JRKSERIAL_H

#include <stdexcept>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <serial/serial.h>
#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>
#include <ros/ros.h>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

class JrkSerial
{
  private:
    int fd;
    boost::scoped_ptr<serial::Serial> serialPtr_;
    const std::string device_;
    const int speed_;
    const int timeout_;
  public:
    JrkSerial();
    void init();
    int readVariable(const unsigned char command);
    bool write(const uint8_t *data, size_t size);
    int readErrors(unsigned char command);
    int readFeedback();
    int readScaledFeedback();
    int readDutyCycle();
    int readTarget();
    int setTarget(unsigned short target);
    int getErrors();
    void printErrors(int errors);
    int clearErrors();
    void closeDevice();
};
#endif  // JRK_INTERFACE_JRKSERIAL_H

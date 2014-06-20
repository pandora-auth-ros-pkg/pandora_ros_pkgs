#ifndef JRK_INTERFACE_JRKSERIAL_H
#define JRK_INTERFACE_JRKSERIAL_H

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
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
  public:
    JrkSerial();
    int readVariable(unsigned char command);
    int readErrors(unsigned char command);
    int readFeedback();
    int readScaledFeedback();
    int readDutyCycle();
    int readTarget();
    int setTarget(unsigned short target);
    int getErrors();
    void printErrors(int errors);
    void clearErrors();
    void closeDevice();
};
#endif  // JRK_INTERFACE_JRKSERIAL_H

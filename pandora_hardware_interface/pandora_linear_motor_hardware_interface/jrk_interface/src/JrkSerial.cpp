#include <jrk_interface/JrkSerial.h>
#include <ros/ros.h>
#include <jrk_interface/JrkDefinitions.h>

JrkSerial::JrkSerial()
{
  // Open the Jrk's virtual COM port.
  const char * device ="/dev/ttyACM0";  // Linux
  fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    perror(device);
    ROS_FATAL("Failed to open the serial port!!!");
    ROS_BREAK();
  }
  ROS_INFO("The serial port is opened.");
  #ifndef _WIN32
  struct termios options;
  tcgetattr(fd, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(fd, TCSANOW, &options);
  #endif
}

void JrkSerial::closeDevice()
{
  close(fd);
}

int JrkSerial::readVariable(unsigned char command)
{
  if (write(fd, &command, 1) == -1)
  {
    perror("error writing");
    return -1;
  }
  unsigned char response[2];
  if (read(fd, response, 2) != 2)
  {
    perror("error reading");
    return -1;
  }
  return response[0] + 256*response[1];
}

int JrkSerial::readFeedback()
{
  return readVariable(FEEDBACK_VARIABLE);
}

int JrkSerial::readScaledFeedback()
{
  return readVariable(SCALED_FEEDBACK_VARIABLE);
}

int JrkSerial::readDutyCycle()
{
  return readVariable(DUTY_CYCLE_VARIABLE);
}

int JrkSerial::readTarget()
{
  return readVariable(TARGET_VARIABLE);
}

int JrkSerial::setTarget(unsigned short target)
{
  unsigned char command[] = {0xC0 + (target & 0x1F), (target >> 5) & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}

int JrkSerial::getErrors()
{
  int errors = readErrors(ERRORS_HALTING_VARIABLE);
  printErrors(errors);
  return errors;
}

int JrkSerial::readErrors(unsigned char command)
{
  if (write(fd, &command, 1) == -1)
  {
    perror("error writing");
    return -1;
  }
  unsigned char response[2];
  if (read(fd, response, 2) != 2)
  {
    perror("error reading");
    return -1;
  }
  return response[0];
}

void JrkSerial::printErrors(int errors)
{
  if ( errors >= 32 && errors <= 63 )
  {
    ROS_ERROR("Feedback disconenct");
    return;
  }
  if ( errors >= 64 && errors <= 127 )
  {
     ROS_ERROR("Maximum current exceeded");
     return;
  }
  switch (errors)
  {
    case 1:
      ROS_ERROR("Awaiting command");
      break;
    case 2:
      ROS_ERROR("No power");
      break;
    case 3:
      ROS_ERROR("Awaiting Command and No power");
      break;
    case 4:
      ROS_ERROR("Motor driver error");
    case 5:
      ROS_ERROR("Awaiting Command and Motor driver error");
    case 6:
      ROS_ERROR("No power and Motor driver error");
      break;
    case 7:
      ROS_ERROR("Awaiting command and No power and Motor driver error");
      break;
    case 8:
      ROS_ERROR("Input invalid");
      break;
    default:
      ROS_ERROR("No errors");
  }
}

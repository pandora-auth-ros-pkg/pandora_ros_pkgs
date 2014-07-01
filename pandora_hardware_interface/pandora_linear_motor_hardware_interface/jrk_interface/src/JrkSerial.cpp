#include <jrk_interface/JrkSerial.h>
#include <ros/ros.h>
#include <jrk_interface/JrkDefinitions.h>

JrkSerial::JrkSerial():
  serialPtr_(NULL),
  device_("/dev/linear"),
  speed_(115200),
  timeout_(100)
{
}	

void JrkSerial::init()
{
	if (serialPtr_ == NULL)
   {
    try
    {
    serialPtr_.reset(
      new serial::Serial(
        device_,
        speed_,
        serial::Timeout::simpleTimeout(timeout_)));
    }
    catch (serial::IOException& ex)
    {
      ROS_FATAL("Cannot open port!!");
      ROS_FATAL("%s", ex.what());
      exit(-1);
    }
  }
  else
  {
    throw std::logic_error("Init called twice!!");
  }
  clearErrors();
}

void JrkSerial::closeDevice()
{
   serialPtr_->close();
}

int JrkSerial::readVariable(const unsigned char command)
{
  uint8_t message[] = {command};
  if ( !write(message, 1) )
  {
    ROS_ERROR_STREAM("error writing: " << strerror(errno));
    //return -1;
  }
  uint8_t response[2];
  if ( serialPtr_->read(response, 2) != 2 )
  {
    ROS_ERROR_STREAM("error reading: " << strerror(errno));
    return -1;
  }
  return response[0] + 256*response[1];
}

bool JrkSerial::write(const uint8_t *data, size_t size)
{
  serialPtr_->flushInput();
  if (serialPtr_->write(data, size) == size)
  {
    return true;
  }
  else
  {
    return false;
  }
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
  uint8_t command[] = {0xB3, 0xC0 + (target & 0x1F), (target >> 5) & 0x7F};
  if ( !serialPtr_->write(command, sizeof(command)) )
  {
    ROS_ERROR_STREAM("error writing" << strerror(errno));
    return -1;
  }
  return 0;
}

//int JrkSerial::getErrors()
//{
  //int errors = readErrors(ERRORS_HALTING_VARIABLE);
  //printErrors(errors);
  //return errors;
//}

//int JrkSerial::readErrors(unsigned char command)
//{
  //uint8_t message[] = {command};	
  //if ( !write(message, 1) )
  //{
    //ROS_ERROR_STREAM("error writing" << strerror(errno));
    //return -1;
  //}
  //unsigned char response[2];
  //if ( serialPtr_->read(static_cast<uint8_t*>(response), 2) != 2 )
  //{
    //ROS_ERROR_STREAM("error reading" << strerror(errno));
    //return -1;
  //}
  //return response[0];
//}

//void JrkSerial::printErrors(int errors)
//{
  //if ( errors >= 32 && errors <= 63 )
  //{
    //ROS_ERROR("Feedback disconenct");
    //return;
  //}
  //if ( errors >= 64 && errors <= 127 )
  //{
     //ROS_ERROR("Maximum current exceeded");
     //return;
  //}
  //switch (errors)
  //{
    //case 1:
      //ROS_ERROR("Awaiting command");
      //break;
    //case 2:
      //ROS_ERROR("No power");
      //break;
    //case 3:
      //ROS_ERROR("Awaiting Command and No power");
      //break;
    //case 4:
      //ROS_ERROR("Motor driver error");
    //case 5:
      //ROS_ERROR("Awaiting Command and Motor driver error");
    //case 6:
      //ROS_ERROR("No power and Motor driver error");
      //break;
    //case 7:
      //ROS_ERROR("Awaiting command and No power and Motor driver error");
      //break;
    //case 8:
      //ROS_ERROR("Input invalid");
      //break;
    //default:
      //ROS_ERROR("No errors");
  //}
//}

int JrkSerial::clearErrors()
{
  uint8_t command[] = {ERRORS_HALTING_VARIABLE};   //Gets error flags halting and clears any latched errors
  if ( !write(command, 1) )
  {
    ROS_ERROR_STREAM("error writing clearing: " << strerror(errno));
    return -1;
  }
  return 0;
}

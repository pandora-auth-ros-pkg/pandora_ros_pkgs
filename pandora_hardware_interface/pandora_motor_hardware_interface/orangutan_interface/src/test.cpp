#include "orangutan_interface/orangutan_serial_interface.h"

int main(int argc, char* argv[])
{
  pandora_hardware_interface::motor::OrangutanSerialInterface motors("/dev/motors", 9600, 100);
  
  if (argc != 3) {
    std::cerr << "Error: I need 2 speeds!!" << std::endl;
    exit(-1);
  }
  
  motors.init();
  
  int speedLeft = atoi(argv[1]);
  int speedRight = atoi(argv[2]);
  
  motors.setSpeeds(speedLeft, speedRight);
  
  return 0;
}

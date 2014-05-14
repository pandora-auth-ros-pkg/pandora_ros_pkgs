#include "pandora_xmega_hardware_interface/xmega_serial_interface.h"

int main(int argc, char** argv)
{
  pandora_hardware_interface::xmega::XmegaSerialInterface xmega("/dev/ttyUSB0", 115200, 10000);
  xmega.init();
  
  double psu = 0, motor = 0;

  while(1) {
    
    xmega.read();
    xmega.getBatteryData(&psu, &motor);
        
    pandora_hardware_interface::xmega::RangeMap range = xmega.getRangeData();

    std::cout << "psu: " << psu << " " << "motor: " << motor << std::endl;
    for (pandora_hardware_interface::xmega::RangeMap::iterator it=range.begin(); it!=range.end(); ++it)
      std::cout << "i2c_addr: " << it->first <<", value: "<< it->second.sonarRange <<std::endl;

    std::cout << std::endl;
  }
  return 0;
}

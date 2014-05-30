pandora_hardware_interface
===================

Contains packages used to interface with hardware


Installation notes for ARM Hardware Interface:
You must copy pandora_arm_hardware_interface/10-lpc4088_cdc.rules to /etc/udev/rules.d/ 
so the ARM uController USB device is always linked to /dev/head and sudo is not required
to open the device.

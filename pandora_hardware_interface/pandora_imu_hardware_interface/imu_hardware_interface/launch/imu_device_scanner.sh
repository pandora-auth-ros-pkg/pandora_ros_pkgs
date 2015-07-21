#!/bin/bash

if [ -a /dev/trax ]; then
    echo "[imu] Launching pandora_imu_hardware_interface with Trax AHRS"
    device="trax"
elif [ -a /dev/compass ]; then
    echo "[imu] Launching pandora_imu_hardware_interface with Ocean Server Compass"
    device="compass"
else
    echo "[imu] ERROR: Failed to detect imu device. Is it connected?"
    device="N/A"
fi

rosparam set imu/device $device

#!/bin/bash

WARN="\033[1;33m"
RESET="\033[0m"
SUCC="\033[1;32m"

ARGS="-i rpi2"

python transport_image_topics.py ${ARGS} 2>/dev/null
EXIT_STATUS=$?

if [ $EXIT_STATUS -ge 1 ]; then
    echo -e "${WARN}[WARN]: Image topics do not publish yet${RESET}"
else
    echo -e "${SUCC}Succesfully transported image topics running on rpi2${RESET}"
fi

exit $EXIT_STATUS

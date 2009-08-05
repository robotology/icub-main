#!/bin/bash
source ./config.sh
yarp disconnect /${APP_NAME}/v2iCub:o /icub/$1_arm/command:i
yarp disconnect /icub/$1_arm/state:o /${APP_NAME}/fbq:i

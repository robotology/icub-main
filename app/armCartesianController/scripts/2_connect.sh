#!/bin/bash
source ./config.sh
yarp connect /${APP_NAME}/v2iCub:o /icub/$1_arm/command:i
yarp connect /icub/$1_arm/state:o /${APP_NAME}/fbq:i

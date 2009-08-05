#!/bin/bash
source ./config.sh
echo quit | yarp rpc /${APP_NAME}/rpc

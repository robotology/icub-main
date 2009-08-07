#!/bin/bash
source ./config.sh
yarp run --on /${RUN_SRV_CTRL} --as ${APP_NAME} --cmd "iKinArmCtrl --name /${APP_NAME} --arm $1"

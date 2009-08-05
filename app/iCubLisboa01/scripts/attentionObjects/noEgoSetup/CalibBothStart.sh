#!/bin/bash

killall camCalib

source ./../config.sh

./startCamCalibLeft.sh
./startCamCalibRight.sh

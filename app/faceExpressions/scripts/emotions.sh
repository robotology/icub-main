#!/bin/bash

./facedevice.sh $1 &
./emotionInterface.sh $1 &

sleep 1

./connect.sh


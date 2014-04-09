#!/bin/bash
for i in {0..1000}
do
echo "----------iteration $i-----------------------"
echo ">>>>> start cam0 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
icubmoddev --device grabber --subdevice dragonfly2 --width 320 --height 240 --video_type 1 --white_balance 0.525 0.672 --gain 0.240 --shutter 0.870 --name /icub/cam/left --brightness 0.662 --DR2 --stamp --d 0 --sharpness 0.5 --hue 0.48 --gamma 0.4 --saturation 0.260 --framerate 30 --use_network_time 1 &
sleep 2
echo ">>>>> start cam1 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
icubmoddev --device grabber --subdevice dragonfly2 --width 320 --height 240 --video_type 1 --white_balance 0.525 0.672 --gain 0.240 --shutter 0.870 --name /icub/cam/right --brightness 0.662 --DR2 --stamp --d 1 --sharpness 0.5 --hue 0.48 --gamma 0.4 --saturation 0.260 --framerate 30 --use_network_time 1 &
sleep 5
echo ">>>>> killing cam <<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
killall -SIGKILL icubmoddev
sleep 1
killall -SIGKILL icubmoddev
sleep 1
echo "--------------kill finished-----------------"
done

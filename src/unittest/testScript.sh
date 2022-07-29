#sleep 4
yarpdatadumper --name /logFT --overwrite --dir logFT &
#yarpdatadumper --name /logIMU --overwrite --dir logIMU &
sleep 4
yarp connect  /testFT/ADCs/measures:o /logFT &
#yarp connect  /testFT/imu/measures:o /logIMU &
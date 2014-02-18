for (( ; ; ))
do
robot-interface 2>&1 | grep MRCHECK
sleep 10
killall -9 robot-interface
sleep 0.5
done

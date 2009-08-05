#!/bin/bash

function waitforport() {
echo "startup: ---------------------------------"
stat=1
while [ $stat -ne 0 ] 
do
	echo "startup: waiting for port $1"
	ret=`yarp ping $1 2>&1`
	stat=$?
	if [ $stat -eq 0 ] 
	then
		stat=`echo $ret | grep -c fail`
	fi
	if [ $stat -ne 0 ] 
	then
		echo "startup: port $1 status $stat [ $ret ]"
	fi
done

echo "startup: port $1 ready."
echo "startup: ---------------------------------"
echo 
}


echo "audio: ---------------------------------"
audioAnalyserMicrophone.ini
audioAnalyser.exe --file audioAnalyserConfig.ini 




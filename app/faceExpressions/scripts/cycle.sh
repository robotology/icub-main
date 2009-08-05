#!/bin/bash

list="hap evi shy cun sur ang"

while true; do
	for a in $list; do
		echo "set all $a"
		echo "set all $a"  | yarp rpc /icub/face/emotions/in
		sleep 2
	done
done

#!/usr/bin/env bash

function stress()
{
	joints="$1"
	port="$2"

	echo $joints
	echo $port

	for j in $joints
	do
	  echo "get enc $j" | yarp rpc $port
	done
}

while [ true ]
do
  stress "0 1 2 3 4 5" "/icub/head/rpc:i"
  stress "0 1 2" "/icub/torso/rpc:i"
  stress "0 1 2 3 4 5 6" "/icub/left_arm/rpc:i"
done
#!/bin/bash



#set col <color>     set the color of the led
	#!! available only for rfe board !!
	#the available colors are: black, white, red, lime, blue, yellow, cyan, magenta, silver, gray, maroon, olive, green, purple, teal, navy 

#set <part> <emotion>  set a specific emotion for a defined part   
	#the available part are: mou, eli, leb, reb, all
	#the available emotions are: neu, hap, sad, sur, ang, evi, shy, cun


list="black white red lime blue yellow cyan magenta silver gray maroon olive green purple teal navy"

while true; do 
	for a in $list; do
                echo "set all hap" | yarp rpc /icub/face/emotions/in
		echo "set col $a"  | yarp rpc /icub/face/emotions/in
		sleep 2
	done
done

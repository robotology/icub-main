#!/bin/bash

source ./config.sh

case "$1" in
    stop)
	killall iCubDemoY3
        ;;
    start)
	list="
$POS_PATH/calibrationSequence1.txt
$POS_PATH/calibrationSequence2.txt
$POS_PATH/calibrationSequence3.txt
$POS_PATH/calibrationSequence4.txt
$POS_PATH/calibrationSequence5.txt
$POS_PATH/calibrationSequence6.txt
$POS_PATH/calibrationSequence7.txt
$POS_PATH/calibrationSequence8.txt
$POS_PATH/calibrationSequence9.txt
$POS_PATH/calibrationSequence10.txt
$POS_PATH/calibrationSequence11.txt
$POS_PATH/calibrationSequence12.txt
$POS_PATH/calibrationSequence13.txt
$POS_PATH/calibrationSequence14.txt
$POS_PATH/calibrationSequence15.txt
$POS_PATH/calibrationSequence16.txt
$POS_PATH/calibrationSequence17.txt
$POS_PATH/calibrationSequence18.txt
$POS_PATH/calibrationSequence19.txt
"

	
	for a in $list; do
	    sleep 1
	    echo "switching sequence $a..."
	    sleep 1
            echo "using $a"
	    command="iCubDemoY3 --positions $a"
            bash -c "$command &"
	    sleep 120
	    killall iCubDemoY3
	done
	
	;;
    *)
        echo "stop, start"
        ;;
esac
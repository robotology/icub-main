#!/bin/bash -l

### Simple startup script for "yarp run".
# Assume environment variables are correctly set
# to point to your YARP binary directory.
# --nat

ID=/`uname -n`

if [ "k$YARP_DIR" != "k" ];
	YARP_DIR=/usr
fi

case "$1" in
	start)
		if [ "k$3" != "k" ];
			then
			disp=$3
		else
			disp=":0.0"
		fi

		if [ "k$2" == "kdisplay" ];
			then
			echo "enabling display on $ID"
			export DISPLAY=$disp
			xhost +
			echo "Display set to: $DISPLAY"
		fi
		
		
		echo "Starting up yarp run for $ID"
		cp -f $YARP_DIR/bin/yarprun /tmp/yarprun
		cd /tmp;./yarprun --server $ID 2>&1 2>/tmp/yarprunserver.log &
		echo \"`date` starting yarprun\" >> /tmp/yarprun.log
		echo "done!"
		;;
	
	kill)
		echo "Killing yarprun for $ID"
		killall -9 yarprun
		echo \"`date` killed yarprun\" >> /tmp/yarprun.log
		echo "done!"
		;;

	stop)
		echo "Stopping yarp run for $ID"
		cd /tmp;./yarprun --exit --on $ID
		echo \"`date` stopped yarprun\" >> /tmp/yarprun.log
		echo "done!"
		;;
esac

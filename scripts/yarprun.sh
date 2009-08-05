#!/bin/bash -l

### Simple startup script for "yarp run".
# Assume environment variables are correctly set
# to point to your YARP binary directory.
# --nat

ID=/`uname -n`
    
cmd="bash -c"; 

case "$1" in
	start)
		if [ k$2 == "kdisplay" ];
			then
			echo "enabling display on $ID"
			export DISPLAY=:0.0
			xhost +
		fi
		
		echo "Starting up yarp run for $ID"
		$cmd "cp -f $YARP_DIR/bin/yarprun /tmp/yarprun"
		$cmd "cd /tmp;./yarprun --server $ID &"
		$cmd "echo \"`date` starting yarprun\" >> /tmp/yarprun.log"
		echo "done!"
		;;
	
	kill)
		echo "Killing yarprun for $ID"
		$cmd "killall -9 yarprun"
		$cmd "echo \"`date` killed yarprun\" >> /tmp/yarprun.log"
		echo "done!"
		;;

	stop)
		echo "Stopping yarp run for $ID"
		$cmd "cd /tmp;./yarprun --exit --on $ID"
		$cmd "echo \"`date` stopped yarprun\" >> /tmp/yarprun.log"
		echo "done!"
		;;
esac

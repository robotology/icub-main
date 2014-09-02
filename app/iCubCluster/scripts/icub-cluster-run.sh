
#!/bin/bash -l

### Simple startup script for "yarp run".
# Assume environment variables are correctly set
# to point to your YARP binary directory.
# --nat

ID=/`uname -n`

if [ "k$YARP_DIR" = "k" ];
then
	YARP_DIR=/usr
fi

if [[ $# -eq 0 ]] ; then
    echo "No argument provided! Here's a list of the available arguments:"
    echo "    start -> Starts up a yarp run. Some arguments can be provided, e.g."
    echo "             When calling  icub-cluster-run.sh  start  display  :0.0 "
    echo "    kill  -> Kills the yarp run"
    echo "    stop  -> Stops the yarp run"
    exit 0
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
		
        if [ "k$2" == "klog" ];
        then
            echo "enabling logger on $ID"
            log=1
        fi

		if [ "k$4" == "klog" ];
        then
            echo "enabling logger on $ID"
            log=1
        fi

        
		echo "Starting up yarp run for $ID"
		cp -f $YARP_DIR/bin/yarprun /tmp/yarprun
        if [ $log == 1 ]
        then        
            cd /tmp;./yarprun --server $ID --log 2>&1 2>/tmp/yarprunserver.log &
        else
		    cd /tmp;./yarprun --server $ID 2>&1 2>/tmp/yarprunserver.log &
        fi
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

#!/bin/bash -l

### Simple startup script for "yarp server".
# nat

if [ X$USER == "Xicub" ];   
    then 
    cmd="bash -l -c"; 
else    
    cmd="su - icub -c"; 
fi;

case "$1" in
	start)
		echo "Starting up yarp server"
		$cmd "cp -f $YARP_ROOT/bin/yarp /tmp/yarpserver"
		$cmd "/tmp/yarpserver server 2>&1 2>/dev/null &"
		echo "done!"
		;;

	stop)
		echo "Stopping yarp server"
		$cmd "killall yarpserver"
		;;

	kill)
		echo "Killing yarp server"
		$cmd "killall -9 yarpserver"
		;;
	*)
		echo "start stop kill"
		;;
esac

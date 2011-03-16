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
		case "$2" in 
			yarpserver) 
				echo "Starting up yarp server"
				$cmd "cp -f $YARP_DIR/bin/yarpserver /tmp/yarpserver"
				$cmd "/tmp/yarpserver 2>&1 2>/dev/null &"
				echo "done!"
				;;
			*) #yarpserver3 is the default
				echo "Starting up yarp server"
				$cmd "cp -f $YARP_DIR/bin/yarpserver3 /tmp/yarpserver3"
				$cmd "/tmp/yarpserver3 --portdb /tmp/ports.db --subdb /tmp/subs.db 2>&1 2>/dev/null &"
				echo "done!"
				;;
		esac
	
	stop)
		echo "Stopping yarp server"
		$cmd "killall yarpserver3"
		$cmd "killall yarpserver"
		;;

	kill)
		echo "Killing yarp server"
		$cmd "killall -9 yarpserver3"
		$cmd "killall -9 yarpserver"
		;;
	*)
		echo "start stop kill"
		;;
esac

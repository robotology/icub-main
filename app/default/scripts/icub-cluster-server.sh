#!/bin/bash -l

### Simple startup script for "yarp server".
# nat

cmd="bash -l -c"; 

echo "Parameters: $1 $2"

case "$1" in
	start)
		case "$2" in 
			yarpserver) 
				echo "Starting up yarp server"
				$cmd "cp -f $YARP_DIR/bin/yarpserver /tmp/yarpserver"
				$cmd "/tmp/yarpserver >/dev/null 2>&1 &"
				echo "done!"
				;;
			*) #yarpserver3 is the default
				echo "Starting up yarp server"
				$cmd "cp -f $YARP_DIR/bin/yarpserver3 /tmp/yarpserver3"
				$cmd "/tmp/yarpserver3 --portdb /tmp/ports.db --subdb /tmp/subs.db >/dev/null 2>&1 &"
				echo "done!"
				;;
		esac
		;;
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

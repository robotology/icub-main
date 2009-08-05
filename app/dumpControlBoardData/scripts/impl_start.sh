#!/bin/bash

case "$1" in
	stop)
		echo "Stopping $3"
		yarp run --on $2  --kill $3
		;;
	start)
		echo "Starting $3"
		yarp run --on $2  --as $3 --cmd "$4"
		;;
	*)
	echo "stop, start"
	;;
esac


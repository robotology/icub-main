

#!/bin/bash

MYSHELL="bash -c"

case "$1" in
	kill)
		killall -9 CrawlGeneratorModule
		;;
    	stop)
		killall CrawlGeneratorModule
		;;
 	start)
		$MYSHELL "CrawlGeneratorModule --part left_arm"&
		sleep 1
		$MYSHELL "CrawlGeneratorModule --part right_arm"&
		sleep 1
		$MYSHELL "CrawlGeneratorModule --part left_leg"&
		sleep 1
		$MYSHELL "CrawlGeneratorModule --part right_leg"&
		sleep 1
		$MYSHELL "CrawlGeneratorModule --part torso"&
        sleep 1
		$MYSHELL "CrawlGeneratorModule --part head"&
		sleep 1
		;;

    *)
		echo "stop, start, kill"
		;;
esac

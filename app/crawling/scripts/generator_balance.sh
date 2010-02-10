

#!/bin/bash

MYSHELL="bash -c"

case "$1" in
	kill)
		killall -9 CrawlGeneratorModule_BalCont
		;;
    	stop)
		killall CrawlGeneratorModule_BalCont
		;;
 	start)
		$MYSHELL "CrawlGeneratorModule_BalCont --part left_arm"&
		sleep .1
		$MYSHELL "CrawlGeneratorModule_BalCont --part right_arm"&
		sleep .1
		$MYSHELL "CrawlGeneratorModule_BalCont --part left_leg"&
		sleep .1
		$MYSHELL "CrawlGeneratorModule_BalCont --part right_leg"&
		sleep .1
		$MYSHELL "CrawlGeneratorModule_BalCont --part torso"&
        	sleep .1
		$MYSHELL "CrawlGeneratorModule_BalCont --part head"&
		sleep .1
		;;

    *)
		echo "stop, start, kill"
		;;
esac

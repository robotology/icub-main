PROT=mcast

machine=`uname -n`

while true; do
	command="framerate --local /$machine/fr1 --remote /testgrabber1 --prot $PROT"
	$command &
	PID1=$!
	command="framerate --local /$machine/fr2 --remote /testgrabber2 --prot $PROT"
	$command &
	PID2=$!
	sleep 10
	killall framerate
	wait $PID1
	echo "Framerate1 returned"
	wait $PID2
	echo "Framerate2 returned"
	echo ""
done


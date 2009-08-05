
op="start"
machine="hercules.james.liralab.it"

# call program with "stop" or "stopmore" as argument to shut things down
if [ ! "k$1" = "k" ]; then
	op=$1
fi

./yarprun $op $machine from-camera1 yarpview --name /bozo/view1 --x 0 --y 400
./yarprun $op $machine from-camera2 yarpview --name /bozo/view2 --x 400 --y 400




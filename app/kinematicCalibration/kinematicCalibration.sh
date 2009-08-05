#!/bin/bash

#source ./config.sh

case "$1" in
    stop)
                killall dataDumper
		command="$ICUB_ROOT/app/kinematicCalibration/demoSwitch.sh stop"
                bash -c "$command"
                ;;
    start)
                

                command="dataDumper --name /dump_head --demul 2"
                bash -c "$command &"
                command="dataDumper --name /dump_left_arm --demul 2"
                bash -c "$command &"
                command="dataDumper --name /dump_right_arm --demul 2"
                bash -c "$command &"
                command="dataDumper --name /dump_torso --demul 2"
                bash -c "$command &"
                command="dataDumper --name /dump_inertial --demul 4"
                bash -c "$command &"
                command="dataDumper --name /dump_left_leg --demul 2"
                bash -c "$command &"
                command="dataDumper --name /dump_right_leg --demul 2"
                bash -c "$command &"
		command="$ICUB_ROOT/app/kinematicCalibration/demoSwitch.sh start"
                bash -c "$command"
                ;;
	connect)    
                command="yarp connect /icub/head/state:o /dump_head udp"
                bash -c "$command &"
                command="yarp connect /icub/left_arm/state:o /dump_left_arm udp"
                bash -c "$command &"
                command="yarp connect /icub/right_arm/state:o /dump_right_arm udp"
                bash -c "$command &"
                command="yarp connect /icub/torso/state:o /dump_torso udp"
                bash -c "$command &"
                command="yarp connect /icub/inertial /dump_inertial udp"
                bash -c "$command &"
                command="yarp connect /icub/left_leg/state:o /dump_left_leg udp"
                bash -c "$command &"
                command="yarp connect /icub/right_leg/state:o /dump_right_leg udp"
                bash -c "$command &"
				;;
	disconnect)    
                command="yarp disconnect /icub/head/state:o /dump_head"
                bash -c "$command &"
                command="yarp disconnect /icub/left_arm/state:o /dump_left_arm"
                bash -c "$command &"
                command="yarp disconnect /icub/right_arm/state:o /dump_right_arm"
                bash -c "$command &"
                command="yarp disconnect /icub/torso/state:o /dump_torso"
                bash -c "$command &"
                command="yarp disconnect /icub/inertial /dump_inertial"
                bash -c "$command &"
                command="yarp disconnect /icub/left_leg/state:o /dump_left_leg"
                bash -c "$command &"
                command="yarp disconnect /icub/right_leg/state:o /dump_right_leg"
                bash -c "$command &"
				;;

    *)
                echo "stop, start, connect, disconnect"
                ;;
esac

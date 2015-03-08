controlBoardDumper --name /rl_dumper --robot icub --part right_leg --joints "(0 1 2 3)" --rate 10 --dataToDump "(getEncoders getEncoderSpeeds getMotorEncoders getMotorEncoderSpeeds)" &
controlBoardDumper --name /ll_dumper --robot icub --part left_leg  --joints "(0 1 2 3)" --rate 10 --dataToDump "(getEncoders getEncoderSpeeds getMotorEncoders getMotorEncoderSpeeds)" &
velocityObserver --name /ll_encs &
velocityObserver --name /ll_motencs &
velocityObserver --name /rl_encs &
velocityObserver --name /rl_motencs &
read -n1 -r -p "press key to continue" key
yarp connect /rl_dumper/right_leg/getMotorEncoders /rl_motencs/pos:i
yarp connect /rl_dumper/right_leg/getEncoders      /rl_encs/pos:i
yarp connect /ll_dumper/left_leg/getMotorEncoders  /ll_motencs/pos:i
yarp connect /ll_dumper/left_leg/getEncoders       /ll_encs/pos:i
read -n1 -r -p "press key to continue" key
yarpscope --xml yarpscope_speeds_ll.xml &
yarpscope --xml yarpscope_speeds_rl.xml &
read -n1 -r -p "press key to terminate" key
killall -9 controlBoardDumper
killall -9 velocityObserver
killall -9 yarpscope


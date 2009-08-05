NOTE: if you want the feedback to be used the midiDrum server should be
 	running. Beats are received from the port midiDrum/server/out.

0. Turn on the velocity control for the right_arm:

	$ICUB_ROOT/src/velocityControl/velocityControl --period 20 --part right_arm

1. Turn on the velocity control for the head:

	$ICUB_ROOT/src/velocityControl/velocityControl --period 20 --part head

2. Turn on the clock 

	$ICUB_ROOT/src/drummingEPFL/GeneralClock$ ./clockModule --period 20 --part head

	(-) Requires a suitable $ICUB_ROOT/src/drummingEPFL/config/headConfig.ini

3. Turn on the drum generator for the right arm:
	
	$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part right_arm --file ../config/right_armConfig.ini

	(-) Requires a suitable $ICUB_ROOT/src/drummingEPFL/config/right_armConfig.ini

4. Turn on the drum manager:

	$ICUB_ROOT/src/drummingEPFL/DrumManager/DrumManager

	(-) Requires a suitable $ICUB_ROOT/src/drummingEPFL/config/right_armTargets.ini
	(-) Requires a suitable $ICUB_ROOT/src/drummingEPFL/config/right_armConfig.ini

5. Turn on the gui:

	$ICUB_ROOT/src/drummingEPFL/guiDemo/guiDemo

	(-) You can specify a score to be executed by a suitable file src/drummingEPFL/guiDemo/part/part1.txt

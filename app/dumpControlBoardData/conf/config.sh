#!/bin/sh

# Attention system configuration file. 
# 10/1/2009 -- Francesco Nori

. ../conf/config-nodes.sh
# Robot Name
NAME_ROBOT=icub
#PARTS_TO_DUMP="head torso left_arm right_arm left_leg right_leg"
PARTS_TO_DUMP="head torso right_arm left_arm right_leg left_leg"

# Paths
EXECUTABLE_PATH=$ICUB_DIR/bin
APPLICATION_PATH=/app/dumpControlBoardData

# Module: controlBoardDumper for head
CONTROL_BOARD_DUMPER_HEAD_EXECUTABLE=controlBoardDumper
CONTROL_BOARD_DUMPER_HEAD_FILE=icubHead.ini
CONTROL_BOARD_DUMPER_HEAD_PART_NAME=head
CONTROL_BOARD_DUMPER_HEAD_NAME=/$NAME_ROBOT/controlBoardDumper/$CONTROL_BOARD_DUMPER_HEAD_PART_NAME
# Module: dataDumper for head
DUMPER_HEAD_EXECUTABLE=dataDumper
DUMPER_HEAD_NAME=/$NAME_ROBOT/dataDumper/head

# Module: controlBoardDumper for head
CONTROL_BOARD_DUMPER_TORSO_EXECUTABLE=controlBoardDumper
CONTROL_BOARD_DUMPER_TORSO_FILE=icubTorso.ini
CONTROL_BOARD_DUMPER_TORSO_PART_NAME=torso
CONTROL_BOARD_DUMPER_TORSO_NAME=/$NAME_ROBOT/controlBoardDumper/$CONTROL_BOARD_DUMPER_TORSO_PART_NAME
# Module: dataDumper for torso
DUMPER_TORSO_EXECUTABLE=dataDumper
DUMPER_TORSO_NAME=/$NAME_ROBOT/dataDumper/torso

# Module: controlBoardDumper for left_arm
CONTROL_BOARD_DUMPER_LEFTARM_EXECUTABLE=controlBoardDumper
CONTROL_BOARD_DUMPER_LEFTARM_FILE=icubLeftArm.ini
CONTROL_BOARD_DUMPER_LEFTARM_PART_NAME=left_arm
CONTROL_BOARD_DUMPER_LEFTARM_NAME=/$NAME_ROBOT/controlBoardDumper/$CONTROL_BOARD_DUMPER_LEFTARM_PART_NAME
# Module: dataDumper for left_arm
DUMPER_LEFTARM_EXECUTABLE=dataDumper
DUMPER_LEFTARM_NAME=/$NAME_ROBOT/dataDumper/left_arm

# Module: controlBoardDumper for right_arm
CONTROL_BOARD_DUMPER_RIGHTARM_EXECUTABLE=controlBoardDumper
CONTROL_BOARD_DUMPER_RIGHTARM_FILE=icubRightArm.ini
CONTROL_BOARD_DUMPER_RIGHTARM_PART_NAME=right_arm
CONTROL_BOARD_DUMPER_RIGHTARM_NAME=/$NAME_ROBOT/controlBoardDumper/$CONTROL_BOARD_DUMPER_RIGHTARM_PART_NAME
# Module: dataDumper for right_arm
DUMPER_RIGHTARM_EXECUTABLE=dataDumper
DUMPER_RIGHTARM_NAME=/$NAME_ROBOT/dataDumper/right_arm

# Module: controlBoardDumper for left_leg
CONTROL_BOARD_DUMPER_LEFTLEG_EXECUTABLE=controlBoardDumper
CONTROL_BOARD_DUMPER_LEFTLEG_FILE=icubLeftLeg.ini
CONTROL_BOARD_DUMPER_LEFTLEG_PART_NAME=left_leg
CONTROL_BOARD_DUMPER_LEFTLEG_NAME=/$NAME_ROBOT/controlBoardDumper/$CONTROL_BOARD_DUMPER_LEFTLEG_PART_NAME
# Module: dataDumper for left_leg
DUMPER_LEFTLEG_EXECUTABLE=dataDumper
DUMPER_LEFTLEG_NAME=/$NAME_ROBOT/dataDumper/left_leg

# Module: controlBoardDumper for right_leg
CONTROL_BOARD_DUMPER_RIGHTLEG_EXECUTABLE=controlBoardDumper
CONTROL_BOARD_DUMPER_RIGHTLEG_FILE=icubRightLeg.ini
CONTROL_BOARD_DUMPER_RIGHTLEG_PART_NAME=right_leg
CONTROL_BOARD_DUMPER_RIGHTLEG_NAME=/$NAME_ROBOT/controlBoardDumper/$CONTROL_BOARD_DUMPER_RIGHTLEG_PART_NAME
# Module: dataDumper for right_leg
DUMPER_RIGHTLEG_EXECUTABLE=dataDumper
DUMPER_RIGHTLEG_NAME=/$NAME_ROBOT/dataDumper/right_leg

# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(icubarmcalibrator TYPE iCubArmCalibrator INCLUDE iCubArmCalibrator.h)
  PREPARE_DEVICE(icubarmcalibratorj8 TYPE iCubArmCalibratorJ8 INCLUDE iCubArmCalibratorJ8.h)
  PREPARE_DEVICE(icubarmcalibratorj4 TYPE iCubArmCalibratorJ4 INCLUDE iCubArmCalibratorJ4.h)
  PREPARE_DEVICE(DFKI_hand_calibrator TYPE DFKI_hand_calibrator INCLUDE DFKI_hand_calibrator.h)
ENDIF (COMPILE_DEVICE_LIBRARY)


IF (NOT SKIP_icubarmcalibratorj8 OR NOT SKIP_icubarmcalibratorj4 OR NOT SKIP_icubarmcalibrator OR NOT SKIP_DFKI_hand_calibrator)

    ADD_DEFINITIONS(-D_REENTRANT)
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${ACE_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

    yarp_add_plugin(armcalibrator iCubArmCalibrator.cpp iCubArmCalibrator.h)
    yarp_add_plugin(armcalibratorj8 iCubArmCalibratorJ8.cpp iCubArmCalibratorJ8.h)
    yarp_add_plugin(armcalibratorj4 iCubArmCalibratorJ4.cpp iCubArmCalibratorJ4.h)
    yarp_add_plugin(DFKI_hand_calibrator DFKI_hand_calibrator.cpp DFKI_hand_calibrator.h)

    target_link_libraries(armcalibratorj8 ${YARP_LIBRARIES})
    target_link_libraries(armcalibratorj4 ${YARP_LIBRARIES})
    target_link_libraries(armcalibrator ${YARP_LIBRARIES})
    target_link_libraries(DFKI_hand_calibrator ${YARP_LIBRARIES})
    icub_export_library(armcalibratorj8)
    icub_export_library(armcalibratorj4)
    icub_export_library(armcalibrator)
    icub_export_library(DFKI_hand_calibrator)

endif (NOT SKIP_icubarmcalibratorj8 OR NOT SKIP_icubarmcalibratorj4 OR NOT SKIP_icubarmcalibrator OR NOT SKIP_DFKI_hand_calibrator)



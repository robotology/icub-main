# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

ADD_DEFINITIONS(-D_REENTRANT)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${ACE_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

IF(COMPILE_DEVICE_LIBRARY)
  yarp_prepare_device(icubarmcalibrator TYPE yarp::dev::iCubArmCalibrator INCLUDE iCubArmCalibrator.h)
ENDIF(COMPILE_DEVICE_LIBRARY)

if(NOT SKIP_icubarmcalibrator)
    yarp_add_plugin(armcalibrator iCubArmCalibrator.cpp iCubArmCalibrator.h)
    target_link_libraries(armcalibrator ${YARP_LIBRARIES})
    icub_export_plugin(armcalibrator)
    yarp_install(FILES icubarmcalibrator.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif(NOT SKIP_icubarmcalibrator)

IF(COMPILE_DEVICE_LIBRARY)
  yarp_prepare_device(icubarmcalibratorj8 TYPE yarp::dev::iCubArmCalibratorJ8 INCLUDE iCubArmCalibratorJ8.h)
ENDIF(COMPILE_DEVICE_LIBRARY)

if(NOT SKIP_icubarmcalibratorj8)
    yarp_add_plugin(armcalibratorj8 iCubArmCalibratorJ8.cpp iCubArmCalibratorJ8.h)
    target_link_libraries(armcalibratorj8 ${YARP_LIBRARIES})
    icub_export_plugin(armcalibratorj8)
    yarp_install(FILES icubarmcalibratorj8.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif(NOT SKIP_icubarmcalibratorj8)

IF(COMPILE_DEVICE_LIBRARY)
  yarp_prepare_device(DFKI_hand_calibrator TYPE yarp::dev::DFKI_hand_calibrator INCLUDE DFKI_hand_calibrator.h)
ENDIF(COMPILE_DEVICE_LIBRARY)

if(NOT SKIP_DFKI_hand_calibrator)
    yarp_add_plugin(DFKI_hand_calibrator DFKI_hand_calibrator.cpp DFKI_hand_calibrator.h)
    target_link_libraries(DFKI_hand_calibrator ${YARP_LIBRARIES})
    icub_export_plugin(DFKI_hand_calibrator)
    yarp_install(FILES DFKI_hand_calibrator.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
endif(NOT SKIP_DFKI_hand_calibrator)


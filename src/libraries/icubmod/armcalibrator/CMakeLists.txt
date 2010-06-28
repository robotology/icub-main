
IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(icubarmcalibrator TYPE iCubArmCalibrator INCLUDE iCubArmCalibrator.h)
  PREPARE_DEVICE(icubarmcalibratorj8 TYPE iCubArmCalibratorJ8 INCLUDE iCubArmCalibratorJ8.h)
  PREPARE_DEVICE(icubarmcalibratorj4 TYPE iCubArmCalibratorJ4 INCLUDE iCubArmCalibratorJ4.h)
ENDIF (COMPILE_DEVICE_LIBRARY)


IF (NOT SKIP_icubarmcalibratorj8 OR NOT SKIP_icubarmcalibratorj4 OR NOT SKIP_icubarmcalibrator)

    ADD_DEFINITIONS(-D_REENTRANT)
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${ACE_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

    add_library(armcalibrator iCubArmCalibrator.cpp iCubArmCalibrator.h)
    add_library(armcalibratorj8 iCubArmCalibratorJ8.cpp iCubArmCalibratorJ8.h)
    add_library(armcalibratorj4 iCubArmCalibratorJ4.cpp iCubArmCalibratorJ4.h)

    target_link_libraries(armcalibratorj8 ${YARP_LIBRARIES})
    target_link_libraries(armcalibratorj4 ${YARP_LIBRARIES})
    target_link_libraries(armcalibrator ${YARP_LIBRARIES})
    icub_export_library(armcalibratorj8)
    icub_export_library(armcalibratorj4)
    icub_export_library(armcalibrator)

endif (NOT SKIP_icubarmcalibratorj8 OR NOT SKIP_icubarmcalibratorj4 OR NOT SKIP_icubarmcalibrator)



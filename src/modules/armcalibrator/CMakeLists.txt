FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(icubarmcalibrator TYPE iCubArmCalibrator INCLUDE iCubArmCalibrator.h)
  PREPARE_DEVICE(icubarmcalibratorj8 TYPE iCubArmCalibratorJ8 INCLUDE iCubArmCalibratorJ8.h)
ENDIF (COMPILE_DEVICE_LIBRARY)

ADD_LIBRARY(armcalibrator iCubArmCalibrator.cpp iCubArmCalibrator.h)
ADD_LIBRARY(armcalibratorj8 iCubArmCalibratorJ8.cpp iCubArmCalibratorJ8.h)


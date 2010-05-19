
IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(icubhandcalibrator TYPE iCubHandCalibrator INCLUDE iCubHandCalibrator.h)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_icubhandcalibrator)

FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

ADD_LIBRARY(handcalibrator iCubHandCalibrator.cpp iCubHandCalibrator.h)

ENDIF (NOT SKIP_icubhandcalibrator)
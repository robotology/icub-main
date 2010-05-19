
IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(icublegscalibrator TYPE iCubLegsCalibrator INCLUDE iCubLegsCalibrator.h)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_icublegscalibrator)

FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

ADD_LIBRARY(legscalibrator iCubLegsCalibrator.cpp iCubLegsCalibrator.h)

ENDIF(NOT SKIP_icublegscalibrator)

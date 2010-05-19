
IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(icubheadcalibrator TYPE iCubHeadCalibrator INCLUDE iCubHeadCalibrator.h)
ENDIF (COMPILE_DEVICE_LIBRARY)


IF (NOT SKIP_icubheadcalibrator)

FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

ADD_LIBRARY(headcalibrator iCubHeadCalibrator.cpp iCubHeadCalibrator.h)

ENDIF(NOT SKIP_icubheadcalibrator)

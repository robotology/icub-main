
IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(icubtorsoonlycalibrator TYPE iCubTorsoOnlyCalibrator INCLUDE iCubTorsoOnlyCalibrator.h)
ENDIF (COMPILE_DEVICE_LIBRARY)


IF (NOT SKIP_icubtorsoonlycalibrator)

FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})


ADD_LIBRARY(torsoonlycalibrator iCubTorsoOnlyCalibrator.cpp iCubTorsoOnlyCalibrator.h)

ENDIF(NOT SKIP_icubtorsoonlycalibrator)

PREPARE_DEVICE(icubheadcalibrator TYPE iCubHeadCalibrator INCLUDE iCubHeadCalibrator.h)

if (NOT SKIP_icubheadcalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
ADD_LIBRARY(headcalibrator iCubHeadCalibrator.cpp iCubHeadCalibrator.h)
target_link_libraries(headcalibrator ${YARP_LIBRARIES})

icub_export_library(headcalibrator)

endif(NOT SKIP_icubheadcalibrator)

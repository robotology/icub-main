PREPARE_DEVICE(icubheadcalibrator TYPE iCubHeadCalibrator INCLUDE iCubHeadCalibrator.h)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
ADD_LIBRARY(headcalibrator iCubHeadCalibrator.cpp iCubHeadCalibrator.h)
target_link_libraries(headcalibrator ${YARP_LIBRARIES})

icub_export_library(headcalibrator "" include/iCub "")
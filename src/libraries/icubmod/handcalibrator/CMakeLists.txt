PREPARE_DEVICE(icubhandcalibrator TYPE iCubHandCalibrator INCLUDE iCubHandCalibrator.h)

if (NOT SKIP_icubhandcalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
ADD_LIBRARY(handcalibrator iCubHandCalibrator.cpp iCubHandCalibrator.h)
target_link_libraries(handcalibrator ${YARP_LIBRARIES})

icub_export_library(handcalibrator)
    
endif(NOT SKIP_icubhandcalibrator)

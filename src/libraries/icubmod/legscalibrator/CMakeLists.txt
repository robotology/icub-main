PREPARE_DEVICE(icublegscalibrator TYPE iCubLegsCalibrator INCLUDE iCubLegsCalibrator.h)


if (NOT SKIP_icublegscalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
ADD_LIBRARY(legscalibrator iCubLegsCalibrator.cpp iCubLegsCalibrator.h)

target_link_libraries(legscalibrator ${YARP_LIBRARIES})

icub_export_library(legscalibrator)

endif (NOT SKIP_icublegscalibrator)

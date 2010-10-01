PREPARE_DEVICE(icubheadcalibratorV2 TYPE iCubHeadCalibratorV2 INCLUDE iCubHeadCalibratorV2.h)

if (NOT SKIP_icubheadcalibratorV2)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
ADD_LIBRARY(headcalibratorV2 iCubHeadCalibratorV2.cpp iCubHeadCalibratorV2.h)
target_link_libraries(headcalibratorV2 ${YARP_LIBRARIES})

icub_export_library(headcalibratorV2)

endif(NOT SKIP_icubheadcalibratorV2)

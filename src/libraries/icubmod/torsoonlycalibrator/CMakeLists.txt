PREPARE_DEVICE(icubtorsoonlycalibrator TYPE iCubTorsoOnlyCalibrator INCLUDE iCubTorsoOnlyCalibrator.h)

if (NOT SKIP_icubtorsoonlycalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(torsoonlycalibrator iCubTorsoOnlyCalibrator.cpp iCubTorsoOnlyCalibrator.h)
target_link_libraries(torsoonlycalibrator ${YARP_LIBRARIES})

icub_export_library(torsoonlycalibrator "" include/iCub "")

endif (NOT SKIP_icubtorsoonlycalibrator)

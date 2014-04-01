# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME virtualAnalogServer)
prepare_device(${PROJECTNAME} TYPE VirtualAnalogServer
               INCLUDE ${PROJECTNAME}.h)


if (NOT SKIP_${PROJECTNAME})
    include_directories(${CMAKE_SOURCE_DIR}/src/libraries/icubmod/debugStream/ )
    include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS})

    yarp_add_plugin(${PROJECTNAME} ${PROJECTNAME}.cpp ${PROJECTNAME}.h)
    target_link_libraries(${PROJECTNAME}  debugStream ${YARP_LIBRARIES})
    icub_export_library(${PROJECTNAME})
endif (NOT SKIP_${PROJECTNAME})

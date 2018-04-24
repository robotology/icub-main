# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME ethLoaderLib)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)


include_directories(${ACE_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR}/canProtocolLib)
include_directories(${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/plus/comm-v2/icub/)
include_directories(${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/core/core)

set(folder_source   ${folder_source}
                    ${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/plus/comm-v2/icub/EoBoards.c
                    ${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/plus/comm-v2/icub/EoUpdaterProtocol.c
                    ${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/core/core/EoCommon.c
    )


source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})
add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${ACE_LIBRARIES})

icub_export_library(ethLoaderLib)


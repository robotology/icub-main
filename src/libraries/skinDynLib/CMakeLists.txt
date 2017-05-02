# Copyright: (C) 2011 RobotCub Consortium
# Authors: Andrea Del Prete & Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME skinDynLib)

project(${PROJECTNAME})

set (SKIN_NAO OFF CACHE BOOL "Build the library for using the skin of Nao robot")
if (SKIN_NAO)
    add_definitions(-DSKIN_NAO)
endif (SKIN_NAO)

set(folder_source   src/skinContact.cpp
                    src/skinContactList.cpp
                    src/dynContact.cpp
                    src/dynContactList.cpp
                    src/common.cpp 
                    src/Taxel.cpp
                    src/skinPart.cpp
                    src/iCubSkin.cpp)
set(folder_header   include/iCub/skinDynLib/skinContact.h
                    include/iCub/skinDynLib/skinContactList.h
                    include/iCub/skinDynLib/dynContact.h
                    include/iCub/skinDynLib/dynContactList.h
                    include/iCub/skinDynLib/common.h
                    include/iCub/skinDynLib/rpcSkinManager.h 
                    include/iCub/skinDynLib/Taxel.h
                    include/iCub/skinDynLib/skinPart.h
                    include/iCub/skinDynLib/iCubSkin.h )

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include                    
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib)

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS ctrlLib
                                   DESTINATION include/iCub/skinDynLib
                                   FILES ${folder_header})



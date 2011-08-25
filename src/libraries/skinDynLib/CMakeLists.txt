# Copyright: (C) 2011 RobotCub Consortium
# Authors: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME skinDynLib)

project(${PROJECTNAME})

set(folder_source       src/skinContact.cpp
                        src/dynContact.cpp
                        src/dynContactList.cpp
                        src/common.cpp )
set(folder_header       include/iCub/skinDynLib/skinContact.h
                        include/iCub/skinDynLib/dynContact.h
                        include/iCub/skinDynLib/dynContactList.h
                        include/iCub/skinDynLib/common.h )

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib)

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS ctrlLib
                                   DESTINATION include/iCub/skinDynLib
                                   FILES ${folder_header})

								   
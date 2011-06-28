# Copyright: (C) 2011 RobotCub Consortium
# Authors: Ilaria Gori, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME pmp)

project(${PROJECTNAME})

set(folder_source src/pmp_helpers.cpp
                  src/pmp_server.cpp
                  src/pmp_client.cpp)
				  
set(folder_header_private include/iCub/pmp/private/pmp_helpers.h)

set(folder_header include/iCub/pmp/pmp.h
                  include/iCub/pmp/pmp_server.h
                  include/iCub/pmp/pmp_client.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header_private} ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${folder_source} ${folder_header_private} ${folder_header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib icubmod)

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS ctrlLib
                                   DESTINATION include/iCub/pmp
                                   FILES ${folder_header})

								   
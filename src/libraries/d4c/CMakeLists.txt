# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ilaria Gori, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME d4c)

project(${PROJECTNAME})

set(folder_source src/d4c_helpers.cpp
                  src/d4c_server.cpp
                  src/d4c_client.cpp)

set(folder_header_private include/iCub/d4c/private/d4c_helpers.h)

set(folder_header include/iCub/d4c/d4c.h
                  include/iCub/d4c/d4c_server.h
                  include/iCub/d4c/d4c_client.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header_private} ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ctrlLib_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${folder_source} ${folder_header_private} ${folder_header})
target_link_libraries(${PROJECTNAME} ctrlLib ${GSL_LIBRARIES} ${YARP_LIBRARIES})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS ctrlLib
                                   DESTINATION include/iCub/d4c
                                   FILES ${folder_header})


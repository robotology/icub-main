# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME ethLoader)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${ACE_INCLUDE_DIRS})
include_directories(${GtkPlus_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} 
                      ${GtkPlus_LIBRARIES}
                      ${YARP_LIBRARIES}
                      ${ACE_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)


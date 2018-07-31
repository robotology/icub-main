# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(PROJECTNAME objectsPropertiesCollector)
project(${PROJECTNAME})

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)



#
# Copyright (C) 2010 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME ctpService)
project(${PROJECTNAME})

set(folder_source main.cpp)

add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} ctrlLib ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)







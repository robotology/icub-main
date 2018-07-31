# Copyright: (C) 2012 RobotCub Consortium
# Authors: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
set(PROJECTNAME imageSplitter)

set(source src/main.cpp src/imageSplitter.cpp)
set(header include/imageSplitter.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(${PROJECTNAME} ${source} ${header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)


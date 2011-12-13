# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Matteo Taiana, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

file(GLOB folder_source *.cpp)
file(GLOB folder_header ../include/iCub/*.hpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(pf3dTracker ${folder_header} ${folder_source})
target_link_libraries(pf3dTracker ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS pf3dTracker DESTINATION bin)


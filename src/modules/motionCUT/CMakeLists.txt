# Copyright: 2010 RobotCub Consortium
# Author: Carlo Ciliberto, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(PROJECTNAME motionCUT)
project(${PROJECTNAME})

set(source_files main.cpp)
set(doc_files ${PROJECTNAME}.xml)
source_group("Source Files" FILES ${source_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${source_files} ${doc_files})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${ICUB_MODULES_INSTALL_DIR})

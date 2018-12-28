# Copyright: 2010 RobotCub Consortium
# Author: Carlo Ciliberto, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

project(motionCUT)

find_package(YARP COMPONENTS OS sig cv)

set(source_files main.cpp)
set(doc_files ${PROJECT_NAME}.xml)
source_group("DOC Files" FILES ${doc_files})

add_executable(${PROJECT_NAME} ${source_files} ${doc_files})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${ICUB_MODULES_INSTALL_DIR})

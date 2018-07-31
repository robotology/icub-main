# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
set(PROJECTNAME lumaChroma)
project(${PROJECTNAME})

# Search for source code.
set(doc_files ${PROJECTNAME}.xml)
source_group("DOC Files"    FILES ${doc_files})

file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/iCub/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

yarp_install(FILES ${doc_files} DESTINATION ${ICUB_MODULES_INSTALL_DIR})


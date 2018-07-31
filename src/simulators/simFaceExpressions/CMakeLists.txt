# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
set(PROJECTNAME simFaceExpressions)
project(${PROJECTNAME})

# Search for source code.
file(GLOB folder_source *.cpp *.cc *.c)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME frameGrabberGui2-gtk)
project(${PROJECTNAME})

file(GLOB_RECURSE folder_header include/*.h)
file(GLOB_RECURSE folder_source src/*.cpp)
source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})

add_definitions(${GTK2_DEFINITIONS})
include_directories(${GTK2_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
add_executable(${PROJECTNAME} ${folder_header} ${folder_source})

target_link_libraries(${PROJECTNAME} ${GTK2_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)


if(NOT TARGET frameGrabberGui2-qt OR ICUB_DEFAULT_GTK)
  set_target_properties(frameGrabberGui2-gtk PROPERTIES OUTPUT_NAME frameGrabberGui2)
endif()

# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME iCubSkinGui-gtk)
project(${PROJECTNAME})

file(GLOB_RECURSE folder_header src/include/*.h)
file(GLOB_RECURSE folder_source src/*.cpp)
source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})

add_definitions(${GTK2_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}/src
                    ${ctrlLib_INCLUDE_DIRS}
                    ${GTK2_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
)

add_executable(${PROJECTNAME} ${folder_header} ${folder_source})

target_link_libraries(${PROJECTNAME} ctrlLib
                                     ${GTK2_LIBRARIES}
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)


if(NOT TARGET iCubSkinGui-qt OR ICUB_DEFAULT_GTK)
  set_target_properties(iCubSkinGui-gtk PROPERTIES OUTPUT_NAME iCubSkinGui)
endif()

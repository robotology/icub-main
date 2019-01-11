# Copyright (C) 2010 RobotCub Consortium
# Authors: Andrea del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME skinManagerGui-gtk)
project(${PROJECTNAME})

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/iCub/skinManagerGui/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ICUB_INCLUDE_DIRS}
                    ${GTK2_INCLUDE_DIRS})

# libsigc++ 2.5.1 or later requires c++11. This is automatically
# enabled with CMake 3.1 or later (using FindGTK2.cmake from CMake
# 3.5). With older versions, we enable it here.
if(NOT ${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_LESS 3.1)
  message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_compile_definitions(${PROJECTNAME} PRIVATE ${GTK2_DEFINITIONS})
if(CMAKE_VERSION VERSION_LESS 3.1)
  if(NOT ${GTK2_SIGC++_VERSION_MAJOR}.${GTK2_SIGC++_VERSION_MINOR}.${GTK2_SIGC++_VERSION_MICRO} VERSION_LESS 2.5.1)
    target_compile_definitions(${PROJECTNAME} PRIVATE ${CXX11_FLAGS})
  endif()
endif()

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${GTK2_LIBRARIES}
                                     skinDynLib)

if(WIN32)
    set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
endif(WIN32)

install(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin)

if(NOT TARGET skinManagerGui-qt OR ICUB_DEFAULT_GTK)
  set_target_properties(skinManagerGui-gtk PROPERTIES OUTPUT_NAME skinManagerGui)
endif()

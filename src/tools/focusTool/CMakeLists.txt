# Copyright: (C) 2011 RobotCub Consortium
# Authors: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME focusTool)
project(${PROJECTNAME})

file(GLOB_RECURSE folder_header include/*.h)
file(GLOB_RECURSE folder_source src/*.cpp)
source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})

include_directories(${GTK2_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

# libsigc++ 2.5.1 or later requires c++11. This is automatically
# enabled with CMake 3.1 or later (using FindGTK2.cmake from CMake
# 3.5). With older versions, we enable it here.
if(NOT ${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_LESS 3.1)
  message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()

add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_compile_definitions(${PROJECTNAME} PRIVATE ${GTK2_DEFINITIONS})
if(CMAKE_VERSION VERSION_LESS 3.1)
  if(NOT ${GTK2_SIGC++_VERSION_MAJOR}.${GTK2_SIGC++_VERSION_MINOR}.${GTK2_SIGC++_VERSION_MICRO} VERSION_LESS 2.5.1)
    target_compile_definitions(${PROJECTNAME} PRIVATE ${CXX11_FLAGS})
  endif()
endif()

target_link_libraries(${PROJECTNAME} ${GTK2_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

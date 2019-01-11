# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME iCubSkinGui-gtk)
project(${PROJECTNAME})

file(GLOB_RECURSE folder_header src/include/*.h)
file(GLOB_RECURSE folder_source src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/src
                    ${GTK2_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
)

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

target_link_libraries(${PROJECTNAME} ctrlLib
                                     ${GTK2_LIBRARIES}
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)


if(NOT TARGET iCubSkinGui-qt OR ICUB_DEFAULT_GTK)
  set_target_properties(iCubSkinGui-gtk PROPERTIES OUTPUT_NAME iCubSkinGui)
endif()

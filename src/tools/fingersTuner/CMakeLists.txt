# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME fingersTuner)
project(${PROJECTNAME})

include(YarpInstallationHelpers)
include(YarpIDL)
yarp_idl_to_dir(${PROJECTNAME}.thrift ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB header include/*.h)
file(GLOB source src/*.cpp)
source_group("Header Files" FILES ${header})
source_group("Source Files" FILES ${source})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${ctrlLib_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${header} ${source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib)
install(TARGETS ${PROJECTNAME} DESTINATION bin)



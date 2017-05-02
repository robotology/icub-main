# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(fingersTuner)

include(YarpIDL)
set(idl_files ${PROJECT_NAME}.thrift)
yarp_add_idl(IDL_GEN_FILES ${idl_files})
source_group("IDL Files" FILES ${idl_files})

include_directories(${ctrlLib_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} main.cpp ${IDL_GEN_FILES} ${idl_files})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

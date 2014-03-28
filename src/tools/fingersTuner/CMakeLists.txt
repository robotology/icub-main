# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME fingersTuner)
project(${PROJECTNAME})

include(YarpIDL)

set(header_files ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h)
set(source_files ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp
                 main.cpp)
set(idl_files ${PROJECTNAME}.thrift)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Header Files" FILES ${header_files})
source_group("Source Files" FILES ${source_files})
source_group("IDL Files"    FILES ${idl_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${IDL_GENERATED_CODE_DIR}/include
					${YARP_INCLUDE_DIRS}
					${ctrlLib_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${header_files} ${source_files} ${idl_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib)
install(TARGETS ${PROJECTNAME} DESTINATION bin)



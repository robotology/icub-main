# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME depth2kin)
project(${PROJECTNAME})

include(YarpIDL)

file(GLOB header_files include/*.h)
file(GLOB source_files src/*.cpp)
list(APPEND header_files ${IDL_GENERATED_CODE_DIR}/include/${PROJECTNAME}_IDL.h
                         ${IDL_GENERATED_CODE_DIR}/include/PointReq.h)
list(APPEND source_files ${IDL_GENERATED_CODE_DIR}/src/${PROJECTNAME}_IDL.cpp
                         ${IDL_GENERATED_CODE_DIR}/src/PointReq.cpp)
set(idl_files ${PROJECTNAME}.thrift)
set(doc_files ${PROJECTNAME}.xml)

yarp_idl_to_dir(${idl_files} ${IDL_GENERATED_CODE_DIR})

source_group("Header Files" FILES ${header_files})
source_group("Source Files" FILES ${source_files})
source_group("IDL Files"    FILES ${idl_files})
source_group("DOC Files"    FILES ${doc_files})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${IDL_GENERATED_CODE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${IPOPT_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${iKin_INCLUDE_DIRS}
                    ${optimization_INCLUDE_DIRS}
                    ${learningMachine_INCLUDE_DIRS})
                    
add_executable(${PROJECTNAME} ${header_files} ${source_files} ${idl_files} ${doc_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${GSL_LIBRARIES}
                                     ${IPOPT_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
                                     ctrlLib
                                     iKin
                                     optimization
                                     learningMachine
                                     icubmod)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${ICUB_MODULES_INSTALL_DIR})


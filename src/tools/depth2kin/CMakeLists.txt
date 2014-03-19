# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME depth2kin)
project(${PROJECTNAME})

include(YarpIDL)

file(GLOB headers include/*.h)
file(GLOB sources src/*.cpp)
set(idl ${PROJECTNAME}.thrift)
set(doc ${PROJECTNAME}.xml)

yarp_idl_to_dir(${idl} ${PROJECT_SOURCE_DIR})

source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})
source_group("IDL Files"    FILES ${idl})
source_group("DOC Files"    FILES ${doc})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${IPOPT_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${iKin_INCLUDE_DIRS}
                    ${optimization_INCLUDE_DIRS}
                    ${learningMachine_INCLUDE_DIRS})
                    
add_executable(${PROJECTNAME} ${headers} ${sources} ${idl} ${doc})
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
yarp_install(FILES ${doc} DESTINATION ${ICUB_MODULES_INSTALL_DIR})


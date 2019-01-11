# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(depth2kin)

find_package(YARP COMPONENTS OS sig dev math cv)

file(GLOB header_files include/*.h)
file(GLOB source_files src/*.cpp)
set(idl_files ${PROJECT_NAME}.thrift)
set(doc_files ${PROJECT_NAME}.xml)

yarp_add_idl(IDL_GEN_FILES ${idl_files})

source_group("IDL Files" FILES ${idl_files})
source_group("DOC Files" FILES ${doc_files})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${IPOPT_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${header_files} ${source_files} ${IDL_GEN_FILES} ${idl_files} ${doc_files})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${IPOPT_DEFINITIONS} _USE_MATH_DEFINES)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES}
                                      ${IPOPT_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ctrlLib
                                      iKin
                                      optimization
                                      learningMachine)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${ICUB_MODULES_INSTALL_DIR})


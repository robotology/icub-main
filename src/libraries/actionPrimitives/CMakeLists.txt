# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(actionPrimitives)

set(folder_source src/actionPrimitives.cpp)
set(folder_header include/iCub/action/actionPrimitives.h)

add_library(${PROJECT_NAME} ${folder_source} ${folder_header})
add_library(ICUB::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${PROJECT_NAME} perceptiveModels ${YARP_LIBRARIES})


set_target_properties(${PROJECT_NAME} PROPERTIES
                                      PUBLIC_HEADER "${folder_header}")

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY
          DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER
          DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/action")


icub_install_basic_package_files(${PROJECT_NAME}
                                 DEPENDENCIES "perceptiveModels HINTS \"\${CMAKE_CURRENT_LIST_DIR}/..\""
                                              YARP_os
                                              YARP_sig
                                              YARP_dev)

# Copyright: (C) 2010-2011 RobotCub Consortium
# Authors: Matteo Fumagalli, Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(iDyn)

set(folder_source src/iDyn.cpp
                  src/iDynInv.cpp
                  src/iDynBody.cpp
                  src/iDynTransform.cpp
                  src/iDynContact.cpp)

set(folder_header include/iCub/iDyn/iDyn.h
                  include/iCub/iDyn/iDynInv.h
                  include/iCub/iDyn/iDynBody.h
                  include/iCub/iDyn/iDynTransform.h
                  include/iCub/iDyn/iDynContact.h)

add_library(${PROJECT_NAME} ${folder_source} ${folder_header})
add_library(ICUB::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${PROJECT_NAME} iKin
                                      skinDynLib
                                      ${YARP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES
                                      PUBLIC_HEADER "${folder_header}")

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY
            DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/iDyn")


icub_install_basic_package_files(${PROJECT_NAME}
                                 DEPENDENCIES iKin
                                              skinDynLib
                                              YARP_os
                                              YARP_sig
                                              YARP_dev
                                              YARP_math)

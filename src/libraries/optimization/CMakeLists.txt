# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(optimization)

set(folder_source src/algorithms.cpp
                  src/calibReference.cpp
                  src/affinity.cpp
                  src/neuralNetworks.cpp)
set(folder_header include/iCub/optimization/algorithms.h
                  include/iCub/optimization/matrixTransformation.h
                  include/iCub/optimization/calibReference.h
                  include/iCub/optimization/affinity.h
                  include/iCub/optimization/neuralNetworks.h)

add_library(${PROJECT_NAME} ${folder_source} ${folder_header})
add_library(ICUB::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${IPOPT_DEFINITIONS} _USE_MATH_DEFINES)

target_include_directories(${PROJECT_NAME} PUBLIC  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
                                           PRIVATE ${IPOPT_INCLUDE_DIRS})

set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME} ctrlLib
                                      ${IPOPT_LIBRARIES}
                                      ${YARP_LIBRARIES})
set(OPTIMIZATION_DEPENDENCIES  ctrlLib
                               YARP_os
                               YARP_sig
                               YARP_dev
                               YARP_math
                               IPOPT)

set_target_properties(${PROJECT_NAME} PROPERTIES
                                      PUBLIC_HEADER "${folder_header}")
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY
            DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/optimization")


icub_install_basic_package_files(${PROJECT_NAME}
                                 DEPENDENCIES ${OPTIMIZATION_DEPENDENCIES})

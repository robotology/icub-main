# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(perceptiveModels)

set(folder_source src/ports.cpp
                  src/sensors.cpp
                  src/nodes.cpp
                  src/models.cpp
                  src/springyFingers.cpp
                  src/tactileFingers.cpp)

set(folder_header_private include/iCub/perception/private/ports.h
                          include/iCub/perception/private/models.h)

set(folder_header include/iCub/perception/sensors.h
                  include/iCub/perception/nodes.h
                  include/iCub/perception/models.h
                  include/iCub/perception/springyFingers.h
                  include/iCub/perception/tactileFingers.h)

add_library(${PROJECT_NAME} ${folder_source} ${folder_header_private} ${folder_header})
add_library(ICUB::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${PROJECT_NAME} ctrlLib
                                      learningMachine
                                      ${YARP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES
                                      PUBLIC_HEADER "${folder_header}")
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        RUNTIME
          DESTINATION "${CMAKE_INSTALL_BINDIR}"
        ARCHIVE
          DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY
          DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER
          DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/perception")


icub_install_basic_package_files(${PROJECT_NAME}
                                 INTERNAL_DEPENDENCIES ctrlLib
                                                       learningMachine)

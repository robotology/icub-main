# Copyright (C) 2015
# Author: Marco Randazzo <marco.randazzo@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(dualCamCalib)

find_package(YARP COMPONENTS os sig cv)

set(folder_source src/main.cpp
                  src/spherical_projection.cpp
                  src/DualCamCalibModule.cpp
                  src/CalibToolFactory.cpp
                  src/PinholeCalibTool.cpp
                  src/SphericalCalibTool.cpp)

set(folder_header include/iCub/spherical_projection.h
                  include/iCub/DualCamCalibModule.h
                  include/iCub/CalibToolFactory.h
                  include/iCub/ICalibTool.h
                  include/iCub/PinholeCalibTool.h
                  include/iCub/SphericalCalibTool.h)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)


# Copyright: (C) 2012 RobotCub Consortium
# Authors: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(stereoCalib)

find_package(YARP COMPONENTS os sig dev math)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} ${source} ${header})
if(OpenCV_VERSION_MAJOR GREATER 2)
  target_compile_definitions(${PROJECT_NAME} PRIVATE OPENCV_GREATER_2)
endif()
target_link_libraries(${PROJECT_NAME}
                      YARP::YARP_os
                      YARP::YARP_sig
                      YARP::YARP_dev
                      YARP::YARP_math
                      iKin
                      opencv_core
                      opencv_imgcodecs
                      opencv_imgproc)
if(OpenCV_VERSION_MAJOR GREATER_EQUAL 5)
  target_link_libraries(${PROJECT_NAME} opencv_calib opencv_objdetect)
else()
  target_link_libraries(${PROJECT_NAME} opencv_calib3d)
endif()
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

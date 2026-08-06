# Copyright: (C) 2012 RobotCub Consortium
# Authors: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(stereoCalib)

find_package(YARP COMPONENTS os sig dev math cv)

set(source_files
  src/CalibrationWriter.cpp
  src/FisheyeCalibrationEngine.cpp
  src/main.cpp
  src/stereoCalibModule.cpp
  src/stereoCalibThread.cpp
)

set(header_files
  include/CalibrationTypes.h
  include/CalibrationWriter.h
  include/FisheyeCalibrationEngine.h
  include/stereoCalibModule.h
  include/stereoCalibThread.h
)

add_executable(${PROJECT_NAME} ${source_files} ${header_files})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)
if(OpenCV_VERSION_MAJOR GREATER 2)
  target_compile_definitions(${PROJECT_NAME} PRIVATE OPENCV_GREATER_2)
endif()
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBS} iKin)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

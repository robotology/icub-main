# Copyright: (C) 2012 RobotCub Consortium
# Authors: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
set(PROJECTNAME stereoCalib)
project(${PROJECTNAME})

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

if(OpenCV_VERSION_MAJOR GREATER 2)
  add_definitions(-DOPENCV_GREATER_2)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${iKin_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${source} ${header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} iKin)
install(TARGETS ${PROJECTNAME} DESTINATION bin)


# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

set(PROJECTNAME iKinGazeCtrl)
project(${PROJECTNAME})

set(folder_header include/iCub/gazeNlp.h
                  include/iCub/utils.h
                  include/iCub/solver.h
                  include/iCub/controller.h
                  include/iCub/localizer.h)

set(folder_source src/gazeNlp.cpp
                  src/utils.cpp
                  src/solver.cpp
                  src/controller.cpp
                  src/localizer.cpp
                  src/main.cpp)

source_group("Header Files" FILES ${folder_header})
source_group("Source Files" FILES ${folder_source})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ctrlLib_INCLUDE_DIRS}
                    ${iKin_INCLUDE_DIRS}
                    ${IPOPT_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

if(MSVC)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ctrlLib iKin ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)



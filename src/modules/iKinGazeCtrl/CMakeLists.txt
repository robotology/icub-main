# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

project(iKinGazeCtrl)

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

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ctrlLib_INCLUDE_DIRS}
                    ${iKin_INCLUDE_DIRS}
                    ${IPOPT_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

# import math symbols from standard cmath
add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECT_NAME} ctrlLib iKin ${IPOPT_LIBRARIES} ${YARP_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)


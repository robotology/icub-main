# Copyright: (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Jacopo Losi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(fineCalibrationChecker)
# find YARP
find_package(YARP COMPONENTS os sig dev)
find_package(iCubDev 2.7.0 QUIET)
# set up our program
add_executable(${PROJECT_NAME} 
    ./src/FineCalibrationCheckerModule.h 
    ./src/FineCalibrationCheckerModule.cpp
    ./src/FineCalibrationCheckerThread.h
    ./src/FineCalibrationCheckerThread.cpp
    ./src/main.cpp 

)

# link with YARP libraries
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ICUB::iCubDev)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

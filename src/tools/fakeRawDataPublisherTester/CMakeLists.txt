# Copyright: (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Jacopo Losi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(fakeRawDataPublisherTester)
# find YARP
find_package(YARP COMPONENTS os sig dev)
# set up our program
add_executable(${PROJECT_NAME} ./src/FakeRawDataPublisherTester.h ./src/FakeRawDataPublisherTester.cpp  ./src/main.cpp )

# link with YARP libraries
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ICUB::iCubDev)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

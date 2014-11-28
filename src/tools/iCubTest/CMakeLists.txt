# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME iCubTest)

file(GLOB folder_main_header *.h )
file(GLOB folder_main_source *.cpp )
file(GLOB folder_tests_header testMotors/*.h testMotorsStiction/*.h testMotorsStictionIncremental/*.h testRoie/*.h)
file(GLOB folder_tests_source testMotors/*.cpp testMotorsStiction/*.cpp testMotorsStictionIncremental/*.cpp testRoie/*.cpp)

source_group("Source Files (main)" FILES ${folder_main_source})
source_group("Header Files (main)" FILES ${folder_main_header})
source_group("Source Files (tests)" FILES ${folder_tests_source})
source_group("Header Files (tests)" FILES ${folder_tests_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${iCubDev_INCLUDE_DIRS})
include_directories(${GSL_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_main_source} ${folder_main_header} ${folder_tests_source} ${folder_tests_header})
add_library(${PROJECTNAME}Lib ${folder_main_source} ${folder_main_header} ${folder_tests_source} ${folder_tests_header})
target_link_libraries(${PROJECTNAME}Lib ${YARP_LIBRARIES} icubmod)

target_link_libraries(${PROJECTNAME} ${PROJECTNAME}Lib ${iCubDev_LIBRARIES} ${GSL_LIBRARIES} icubmod ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME}Lib DESTINATION lib)
install(TARGETS ${PROJECTNAME} DESTINATION bin)

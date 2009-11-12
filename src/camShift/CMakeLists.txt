PROJECT(effectDetector)

SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
LINK_LIBRARIES(${OPENCV_LIBRARIES})

SET(folder_source src/yarpTimer.cpp src/effectDetector.cpp src/effectDetectorMain.cpp)
SET(folder_header include/iCub/yarpTimer.h include/iCub/effectDetector.h)
INCLUDE_DIRECTORIES(./include)

ADD_EXECUTABLE(effectDetector ${folder_source} ${folder_header})

install(TARGETS effectDetector DESTINATION bin)

PROJECT(nose_finder)

SET(YARP_DIR "$ENV{YARP_BUILD}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIRECTORY})

ADD_EXECUTABLE(nose_finder main.cpp get_device.cpp get_device.h NoseFinder.h NoseFinder.cpp hough_transform.h hough_transform.cpp)


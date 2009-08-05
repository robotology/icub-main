PROJECT(tracker)

SET(YARP_DIR "$ENV{YARP_BUILD}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
ADD_EXECUTABLE(tracker ImgTrack.cpp attn_tracker.cpp ImgTrack.h TrackerMonitor.cpp TrackerMonitor.h RetinalMap.h)

INSTALL_TARGETS(/bin tracker)

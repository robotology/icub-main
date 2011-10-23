# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Matteo Taiana, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_executable(pf3dTracker pf3dTrackerMain.cpp pf3dTrackerSupport.cpp pf3dTracker.cpp)
target_link_libraries(pf3dTracker ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS pf3dTracker DESTINATION bin)


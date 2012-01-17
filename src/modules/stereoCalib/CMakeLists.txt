# Copyright: (C) 2012 RobotCub Consortium
# Authors: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
SET(PROJECTNAME stereoCalib)

PROJECT(${PROJECTNAME})



FILE(GLOB source src/*.cpp)
FILE(GLOB header include/*.h)

SOURCE_GROUP("Source Files" FILES ${source})
SOURCE_GROUP("Header Files" FILES ${header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})
					
ADD_EXECUTABLE(${PROJECTNAME} ${source} ${header})
TARGET_LINK_LIBRARIES(${PROJECTNAME}  ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)


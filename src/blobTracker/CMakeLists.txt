SET(PROJECTNAME blobTracker)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_DIR}/conf ${ICUB_DIR}/conf)
FIND_PACKAGE(OpenCV REQUIRED)

SET(folder_source main.cpp
                  BlobKalmanFilter.cpp)

SET(folder_header util.h
                  BlobKalmanFilter.h)

SOURCE_GROUP("Source Files" FILES ${folder_source} ${folder_header})

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})
					   
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)




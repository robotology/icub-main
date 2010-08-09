SET(PROJECTNAME blobTracker)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp
                  BlobKalmanFilter.cpp)

SET(folder_header util.h
                  BlobKalmanFilter.h)

SOURCE_GROUP("Source Files" FILES ${folder_source} ${folder_header})

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})
					   
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OPENCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

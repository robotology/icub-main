SET(PROJECTNAME faceTracker)

PROJECT(${PROJECTNAME})

IF(NOT ICUB_USE_OpenCV)
   MESSAGE(FATAL_ERROR "${PROJECTNAME}: OpenCV is strictly required!")
ENDIF(NOT ICUB_USE_OpenCV)

SET(folder_source main.cpp
                  facedet.cpp)

SET(folder_header facedet.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OPENCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)


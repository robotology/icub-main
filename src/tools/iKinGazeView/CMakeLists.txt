SET(PROJECTNAME iKinGazeView)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${ctrlLib_INCLUDE_DIRS}
                    ${MATLAB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
                                     ${MATLAB_LIBRARIES}
                                     ${YARP_LIBRARIES)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)





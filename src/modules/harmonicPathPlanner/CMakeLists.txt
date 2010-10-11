SET(PROJECTNAME harmonicPathPlanner)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp HarmonicFunction.cpp)

SET(folder_header HarmonicFunction.h)

SOURCE_GROUP("Source Files" FILES ${folder_source} ${folder_header})

INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS} )
					   
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${OPENCV_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

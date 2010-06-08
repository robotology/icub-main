SET(PROJECTNAME affActionPrimitives)

PROJECT(${PROJECTNAME})

SET(folder_source src/affActionPrimitives.cpp)
SET(folder_header include/iCub/actions/affActionPrimitives.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ctrlLib
                                      ${YARP_LIBRARIES})
				   				   

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    DESTINATION include/iCub/actions
                                    DEPENDS ctrlLib
                                    FILES ${folder_header})
				   


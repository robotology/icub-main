SET(PROJECTNAME affActionPrimitives)

PROJECT(${PROJECTNAME})

SET(folder_source src/affActionPrimitives.cpp)
SET(folder_header include/iCub/actions/affActionPrimitives.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} iDyn
                                      ${YARP_LIBRARIES})
				   				   
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DESTINATION include/iCub/actions
                                   DEPENDS iDyn
                                   FILES ${folder_header})

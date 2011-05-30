SET(PROJECTNAME perceptiveModels)

PROJECT(${PROJECTNAME})

SET(folder_source src/sensors.cpp src/nodes.cpp src/models.cpp)
SET(folder_header include/iCub/perception/perceptiveModels.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${ctrlLib_INCLUDE_DIRS}
                    ${learningMachine_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ctrlLib
                                      learningMachine
                                      ${YARP_LIBRARIES})
				   				   
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS ctrlLib learningMachine
                                   DESTINATION include/iCub/perception
                                   FILES ${folder_header})


SET(PROJECTNAME iKinArmCtrlIF)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(icubmod
                    ${GSL_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} icubmod
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)





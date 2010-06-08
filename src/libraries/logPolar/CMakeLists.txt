SET(PROJECTNAME logPolar)

PROJECT(${PROJECTNAME})

SET(folder_source src/RC_DIST_FB_logpolar_mapper.cpp)
SET(folder_header include/iCub/RC_DIST_FB_logpolar_mapper.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})
                    
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include"
                                   DESTINATION include/iCub
                                   FILES ${folder_header})

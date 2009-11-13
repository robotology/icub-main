SET(PROJECTNAME affActionPrimitives)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(folder_source src/affActionPrimitives.cpp
                  src/handMetrics.cpp
                  src/common.cpp)
SET(folder_header include/iCub/affActionPrimitives.h
                  include/iCub/handMetrics.h
                  include/iCub/common.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})


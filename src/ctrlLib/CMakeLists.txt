SET(PROJECTNAME ctrlLib)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(folder_source src/ctrlMath.cpp
                  src/filters.cpp
                  src/kalman.cpp
                  src/pids.cpp
                  src/adaptWinPolyEstimator.cpp                  
                  src/trajectoryGenerator.cpp
                  src/functionEncoder.cpp)

SET(folder_header include/iCub/ctrlMath.h
                  include/iCub/filters.h
                  include/iCub/kalman.h
                  include/iCub/pids.h
                  include/iCub/adaptWinPolyEstimator.h                  
                  include/iCub/trajectoryGenerator.h
                  include/iCub/functionEncoder.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})



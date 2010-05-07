SET(PROJECTNAME ctrlLib)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(GSL)

SET(folder_source src/ctrlMath.cpp
                  src/filters.cpp
                  src/kalman.cpp
                  src/pids.cpp
                  src/adaptWinPolyEstimator.cpp                  
                  src/minJerkCtrl.cpp
                  src/functionEncoder.cpp
                  src/optimalControl.cpp)

SET(folder_header include/iCub/ctrlMath.h
                  include/iCub/filters.h
                  include/iCub/kalman.h
                  include/iCub/pids.h
                  include/iCub/adaptWinPolyEstimator.h                  
                  include/iCub/minJerkCtrl.h
                  include/iCub/functionEncoder.h
                  include/iCub/optimalControl.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${GSL_INCLUDE_DIR})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${GSL_LIBRARIES})

IF(WIN32 AND NOT CYGWIN)
   SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
ENDIF(WIN32 AND NOT CYGWIN)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)
INSTALL(FILES ${folder_header} DESTINATION include/iCub)


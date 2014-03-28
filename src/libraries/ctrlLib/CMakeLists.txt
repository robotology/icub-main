# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME ctrlLib)

PROJECT(${PROJECTNAME})

SET(folder_source src/math.cpp
                  src/filters.cpp
                  src/kalman.cpp
                  src/pids.cpp
                  src/tuning.cpp
                  src/adaptWinPolyEstimator.cpp
                  src/minJerkCtrl.cpp
                  src/functionEncoder.cpp
                  src/optimalControl.cpp
                  src/neuralNetworks.cpp
				  src/outliersDetection.cpp)

SET(folder_header include/iCub/ctrl/math.h
                  include/iCub/ctrl/filters.h
                  include/iCub/ctrl/kalman.h
                  include/iCub/ctrl/pids.h
                  include/iCub/ctrl/tuning.h
                  include/iCub/ctrl/adaptWinPolyEstimator.h
                  include/iCub/ctrl/minJerkCtrl.h
                  include/iCub/ctrl/functionEncoder.h
                  include/iCub/ctrl/optimalControl.h
                  include/iCub/ctrl/neuralNetworks.h
				  include/iCub/ctrl/outliersDetection.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})
                 
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DESTINATION include/iCub/ctrl
                                   FILES ${folder_header})


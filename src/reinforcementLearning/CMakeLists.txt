#/*
# * Copyright (C) 2010 Stephen Hart. All rights reserved.
# * 
# */
cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME rl)
PROJECT(${PROJECTNAME})
SET(RL_MODULE reinforcementLearning)

ADD_DEFINITIONS(-O3 -Wno-deprecated -DTIXML_USE_STL)

FIND_PACKAGE(YARP QUIET)
FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(tinyxml .)

ADD_EXECUTABLE(${RL_MODULE} main.cpp MarkovDecisionProcess.cpp tinyxml/tinystr.cpp tinyxml/tinyxml.cpp tinyxml/tinyxmlerror.cpp tinyxml/tinyxmlparser.cpp )
ADD_LIBRARY(${PROJECTNAME} MarkovDecisionProcess.cpp tinyxml/tinystr.cpp tinyxml/tinyxml.cpp tinyxml/tinyxmlerror.cpp tinyxml/tinyxmlparser.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${IKIN_LIBRARIES} ${CTRLLIB_LIBRARIES} ${IPOPTLIB_LIBRARIES} ${REINFORCEMENTLEARNING_LIBRARIES})

INSTALL_TARGETS(/bin ${RL_MODULE})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/lib)


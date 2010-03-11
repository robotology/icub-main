#/*
# * Copyright (C) 2010 Stephen Hart. All rights reserved.
# * 
# */
cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME reinforcementLearning)
PROJECT(${PROJECTNAME})
SET(RL_MODULE reinforcementLearning)

ADD_DEFINITIONS(-O3 -Wno-deprecated -DTIXML_USE_STL)

FIND_PACKAGE(YARP QUIET)
FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(tinyxml)
                    
ADD_EXECUTABLE(reinforcementLearning 
  main.cpp 
  MarkovDecisionProcess.cpp  
  tinyxml/tinyxml.cpp
  tinyxml/tinystr.cpp
  tinyxml/tinyxmlerror.cpp
  tinyxml/tinyxmlparser.cpp
 )

INSTALL_TARGETS(/bin ${RL_MODULE})



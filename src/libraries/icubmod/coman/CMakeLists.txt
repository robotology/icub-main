# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Name of the project must correspond to the class name that needs to be 
# instantiated by the factory

SET(COMAN_MC comanMotionControl)
SET(COMAN_FT comanFTsensor)

PROJECT(COMAN)

PREPARE_DEVICE(${COMAN_MC} TYPE ${COMAN_MC} INCLUDE ${COMAN_MC}.h  WRAPPER controlboard)
PREPARE_DEVICE(${COMAN_FT} TYPE ${COMAN_FT} INCLUDE ${COMAN_FT}.h  WRAPPER analogServer)

IF (NOT SKIP_COMAN )

  if (USE_REALTIME) 
    add_definitions(-DRT_ENV)
  endif (USE_REALTIME)

  #set(CMAKE_INCLUDE_CURRENT_DIR ON)

  set(ThisLib_SOURCES 
  Boards_iface.cpp
  CommProtocol.cpp
  DSP_board.cpp
  comanMotionControl.cpp
  comanFTsensor.cpp
  )

  set(HEADERS_TO_INSTALL
  Boards_iface.h
  CommProtocol.hpp
  definitions.h
  DSP_board.h
  comanMotionControl.h
  comanFTsensor.h
  )

  #add_definitions("-std=c++0x")

  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ../debugStream/)

  add_library(${COMAN_MC} ${ThisLib_SOURCES})
  add_library(${COMAN_FT} ${ThisLib_SOURCES})

  target_link_libraries(${COMAN_MC} yaml-cpp debugStream ${YARP_LIBRARIES} ${ACE_LIBRARIES} )
  target_link_libraries(${COMAN_FT} yaml-cpp debugStream ${YARP_LIBRARIES} ${ACE_LIBRARIES} )

#   install(TARGETS ${THIS_LIB} ARCHIVE DESTINATION lib)
#   install(FILES ${HEADERS_TO_INSTALL} DESTINATION include)

  icub_export_library(${COMAN_MC} )
  icub_export_library(${COMAN_FT} )
ENDIF (NOT SKIP_COMAN )
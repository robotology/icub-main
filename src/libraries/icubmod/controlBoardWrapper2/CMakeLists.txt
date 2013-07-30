# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


prepare_device(controlboardwrapper2 TYPE ControlBoardWrapper2
               INCLUDE ControlBoardWrapper2.h)

prepare_device(controlboardwrapper2wip TYPE ControlBoardWrapper2
               INCLUDE ControlBoardWrapper2wip.h)
	
if (ENABLE_icubmod_controlboardwrapper2 AND ENABLE_icubmod_controlboardwrapper2wip)
	message(FATAL_ERROR "Cannot enable both ENABLE_icubmod_controlboardwrapper2 and ENABLE_icubmod_controlboardwrapper2wip, please choose one")
endif()
	
if (NOT SKIP_controlboardwrapper2)
   include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS})

   add_library(controlboardwrapper2 ControlBoardWrapper2.cpp ControlBoardWrapper2.h)
   target_link_libraries(controlboardwrapper2  ${YARP_LIBRARIES})
   icub_export_library(controlboardwrapper2)
endif()

if (NOT SKIP_controlboardwrapper2wip)
   include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS})

   add_library(controlboardwrapper2 ControlBoardWrapper2wip.cpp ControlBoardWrapper2wip.h)
   target_link_libraries(controlboardwrapper2  ${YARP_LIBRARIES})
   icub_export_library(controlboardwrapper2)
endif()



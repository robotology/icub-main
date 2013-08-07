# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


prepare_device(controlboardwrapper2 TYPE ControlBoardWrapper2
               INCLUDE ControlBoardWrapper2.h ControlBoardWrapper2wip.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS})

if (ICUB_CONTROLWRAPPER_WIP)
   add_library(controlboardwrapper2 ControlBoardWrapper2wip.cpp ControlBoardWrapper2wip.h)
else()
   add_library(controlboardwrapper2 ControlBoardWrapper2.cpp ControlBoardWrapper2.h)
endif()
   target_link_libraries(controlboardwrapper2  ${YARP_LIBRARIES})
   icub_export_library(controlboardwrapper2)




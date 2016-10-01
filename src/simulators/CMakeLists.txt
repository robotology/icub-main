# Copyright: (C) 2009,2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(ICUB_USE_ODE AND ICUB_USE_SDL)
  add_subdirectory(iCubSimulation)
  add_subdirectory(simFaceExpressions)
else()
  message(STATUS "ODE or SDL not found/selected, skipping iCubSimulation and simFaceExpressions")
endif()



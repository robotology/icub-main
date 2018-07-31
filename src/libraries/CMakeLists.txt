# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

##################
# IMPORTANT: the order with which you add directories
# here matters.
# Put libraries in dependency order. If A depends from B
# put B before A. Failure to do so might break the build.
#

add_subdirectory(iCubDev)
add_subdirectory(ctrlLib)
add_subdirectory(skinDynLib)
add_subdirectory(iKin)
add_subdirectory(iDyn)

if(ICUB_USE_GSL)
  add_subdirectory(learningMachine)
else()
  message(STATUS "GSL not found/selected, skipping learningMachine")
endif()

if(TARGET learningMachine)
  add_subdirectory(perceptiveModels)
else()
  message(STATUS "Unmet dependencies, skipping perceptiveModels")
endif()

if((TARGET learningMachine) AND (TARGET perceptiveModels))
  add_subdirectory(actionPrimitives)
else()
  message(STATUS "Unmet dependencies, skipping actionPrimitives")
endif()

add_subdirectory(icubmod)

if(ICUB_USE_IPOPT)
  add_subdirectory(optimization)
else()
  message(STATUS "IPOPT not found/selected, skipping optimization")
endif()

# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

##################
# IMPORTANT: the order with which you add directories 
# here matters. 
# Put libraries in dependency order. If A depends from B
# put B before A. Failure to do so might break the build.
# 

add_subdirectory(iCubIDLClients)
add_subdirectory(iCubDev)
add_subdirectory(iCubInterfaceGuiLib)

if(ICUB_USE_OpenCV)
    add_subdirectory(iCubVis)
else(ICUB_USE_OpenCV)
    message(STATUS "OpenCV not found/selected, skipping iCubVis")
endif(ICUB_USE_OpenCV)

add_subdirectory(logpolar)
add_subdirectory(boostMIL)
add_subdirectory(ctrlLib)
add_subdirectory(skinDynLib)
add_subdirectory(iKin)
add_subdirectory(iDyn)
add_subdirectory(learningMachine)
add_subdirectory(perceptiveModels)
add_subdirectory(actionPrimitives)
add_subdirectory(icubmod)
add_subdirectory(d4c)

if(ICUB_USE_IPOPT)
    add_subdirectory(optimization)
else(ICUB_USE_IPOPT)
    message(STATUS "IPOPT not found/selected, skipping optimization")
endif(ICUB_USE_IPOPT)



# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(ctpService)
add_subdirectory(cartesianInterfaceControl)
add_subdirectory(positionDirectControl)
add_subdirectory(velocityControl)
add_subdirectory(velocityObserver)
add_subdirectory(wholeBodyDynamics)
add_subdirectory(gravityCompensator)
add_subdirectory(skinManager)
add_subdirectory(objectsPropertiesCollector)

if(TARGET learningMachine)
  add_subdirectory(learningMachine)
else()
  message(STATUS "learningMachine not compiled, skipping modules dependant binaries")
endif()

if(ICUB_USE_OpenCV)
    add_subdirectory(camCalib)
    add_subdirectory(camCalibWithPose)
    add_subdirectory(dualCamCalib)
    add_subdirectory(motionCUT)

    if(ICUB_USE_GSL)
        add_subdirectory(templatePFTracker)
    else()
        message(STATUS "GSL not found/selected, skipping templatePFTracker")
    endif()

    if(ICUB_OpenCV_LEGACY)
        message(STATUS "OpenCV legacy detected, skipping lumaChroma")
        # put hereafter modules that rely on old versions of OpenCV declared as legacy
    else()
        add_subdirectory(lumaChroma)
    endif()
else()
    message(STATUS "OpenCV not found/selected, skipping camCalib")
    message(STATUS "OpenCV not found/selected, skipping motionCUT")
    message(STATUS "OpenCV not found/selected, skipping lumaChroma")
    message(STATUS "OpenCV not found/selected, skipping templatePFTracker")
endif(ICUB_USE_OpenCV)

if(ICUB_USE_IPOPT)
    add_subdirectory(iKinGazeCtrl)
    add_subdirectory(iKinCartesianSolver)
else()
    message(STATUS "IPOPT not found/selected, skipping iKinGazeCtrl")
    message(STATUS "IPOPT not found/selected, skipping iKinCartesianSolver")
endif()

if(ICUB_USE_IPOPT AND ICUB_USE_OpenCV AND (TARGET actionPrimitives))
    add_subdirectory(actionsRenderingEngine)
else()
    message(STATUS "Unmet dependencies, skipping actionsRenderingEngine")
endif()

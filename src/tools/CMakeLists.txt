# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(controlBoardDumper)
add_subdirectory(simpleClient)
add_subdirectory(testStereoMatch)
add_subdirectory(fingersTuner)
add_subdirectory(imageSplitter)
add_subdirectory(trajectoryPlayer)
add_subdirectory(imageBlender)
add_subdirectory(imageCropper)
add_subdirectory(embObjProtoTools/boardTransceiver)
add_subdirectory(wholeBodyPlayer)

add_subdirectory(canLoader)
add_subdirectory(ethLoader)
add_subdirectory(strainCalib)

# Tools available both for Qt5
if(ICUB_USE_Qt5)
   if(ICUB_USE_GSL)
     add_subdirectory(iCubSkinGui-qt)
   else()
     message(STATUS "GSL not found, skipping iCubSkinGui-qt")
   endif()

   add_subdirectory(frameGrabberGui2-qt)
   add_subdirectory(skinManagerGui-qt)
   add_subdirectory(FirmwareUpdater)
endif()
if(NOT ICUB_USE_Qt5)
   message(STATUS "Qt5 not found/selected, skipping iCubSkinGui")
   message(STATUS "Qt5 not found/selected, skipping frameGrabberGui2")
   message(STATUS "Qt5 not found/selected, skipping skinManagerGui")
endif()

if(ICUB_USE_SDL)
   add_subdirectory(joystickCtrl)
   add_subdirectory(joystickCheck)
else(ICUB_USE_SDL)
   message(STATUS "SDL not found/selected, skipping joystickCtrl")
endif(ICUB_USE_SDL)

if(ICUB_USE_Qt5 AND ICUB_USE_OpenGL AND ICUB_USE_GLUT)
    add_subdirectory(iCubGui)
else()
    message(STATUS "Qt5 or OpenGL or GLUT not found/selected, skipping iCubGui")
endif()

if(ICUB_USE_OpenCV)
    if(ICUB_OpenCV_LEGACY)
        message(STATUS "OpenCV legacy detected, skipping stereoCalib")
        # put hereafter modules that rely on old versions of OpenCV declared as legacy
    else(ICUB_OpenCV_LEGACY)
        add_subdirectory(stereoCalib)
    endif(ICUB_OpenCV_LEGACY)
else(ICUB_USE_OpenCV)
    message(STATUS "OpenCV not found/selected, skipping stereoCalib")
endif(ICUB_USE_OpenCV)

if(ICUB_USE_IPOPT AND (TARGET learningMachine) AND (ICUB_USE_OpenCV AND (NOT ICUB_OpenCV_LEGACY)))
  add_subdirectory(depth2kin)
else()
  message(STATUS "Unmet dependencies, skipping depth2kin")
endif()

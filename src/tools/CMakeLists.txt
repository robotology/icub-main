# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(controlBoardDumper)
add_subdirectory(dataDumper)
add_subdirectory(simpleClient)
add_subdirectory(testStereoMatch)
add_subdirectory(iCubTest)

#canLoader needs GtkPlus, but it contains a library needed by other modules
add_subdirectory(canLoader)

if(ICUB_USE_GtkPlus)
	add_subdirectory(robotMotorGui)
	add_subdirectory(iCubSkinGui)
else(ICUB_USE_GtkPlus)
	if(ICUB_USE_INTERFACEGUI)
		message(STATUS "GtkPlus not selected, skipping iCubSkinGui")
	endif(ICUB_USE_INTERFACEGUI)
    	message(STATUS "GtkPlus not selected, skipping robotMotorGui")
endif(ICUB_USE_GtkPlus)

if(ICUB_USE_SDL)
	add_subdirectory(joystickCtrl)
else(ICUB_USE_SDL)
	message(STATUS "SDL not selected, skipping joystickCtrl")
endif(ICUB_USE_SDL)

if(ICUB_USE_GtkMM)
	add_subdirectory(frameGrabberGui2)
	if(ICUB_USE_INTERFACEGUI)
	  add_subdirectory(iCubInterfaceGui)
	endif(ICUB_USE_INTERFACEGUI)
else(ICUB_USE_GtkMM)
	message(STATUS "GtkMM not selected, skipping frameGrabberGui2")
	message(STATUS "GtkMM not selected, skipping iCubInterfaceGui")
endif(ICUB_USE_GtkMM)

if(ICUB_USE_Qt3 AND ICUB_USE_OpenGL AND ICUB_USE_GLUT)
	add_subdirectory(iCubGui)
else(ICUB_USE_Qt3 AND ICUB_USE_OpenGL AND ICUB_USE_GLUT)
	message(STATUS "Qt3 or OpenGL or GLUT not selected, skipping iCubGui")
endif(ICUB_USE_Qt3 AND ICUB_USE_OpenGL AND ICUB_USE_GLUT)


if(BUILD_MATLAB_BASED_TOOLS)
	add_subdirectory(iKinArmView)
	add_subdirectory(iKinGazeView)
	add_subdirectory(signalScope)
else(BUILD_MATLAB_BASED_TOOLS)
	message(STATUS "skipping iKinArmView")
	message(STATUS "skipping iKinGazeView")
	message(STATUS "skipping signalScope")
endif(BUILD_MATLAB_BASED_TOOLS)



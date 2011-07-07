# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(controlBoardDumper)
add_subdirectory(dataDumper)
add_subdirectory(simpleClient)
add_subdirectory(testStereoMatch)
add_subdirectory(iCubTest)
add_subdirectory(iSpeak)

#canLoader needs GtkPlus, but it contains a library needed by other modules
add_subdirectory(canLoader)

if(ICUB_USE_GtkPlus)
	add_subdirectory(robotMotorGui)
	add_subdirectory(iCubSkinGui)
	if(ICUB_GtkMM_LEGACY)
		message(STATUS "GtkPlus version before 2.14, skipping skinDriftCompensationGui")
	else(ICUB_GtkMM_LEGACY)
		add_subdirectory(skinDriftCompensationGui)
	endif(ICUB_GtkMM_LEGACY)
else(ICUB_USE_GtkPlus)
	message(STATUS "GtkPlus not selected, skipping iCubSkinGui")
    	message(STATUS "GtkPlus not selected, skipping robotMotorGui")
	message(STATUS "GtkPlus not selected, skipping skinDriftCompensationGui")
endif(ICUB_USE_GtkPlus)

if(ICUB_USE_SDL)
	add_subdirectory(joystickCtrl)
	add_subdirectory(joystickCheck)
else(ICUB_USE_SDL)
	message(STATUS "SDL not selected, skipping joystickCtrl")
endif(ICUB_USE_SDL)

if(ICUB_USE_GtkMM)
	add_subdirectory(frameGrabberGui2)
	add_subdirectory(focusTool)
	if(ICUB_USE_INTERFACEGUI)
	  add_subdirectory(iCubInterfaceGui)
	endif(ICUB_USE_INTERFACEGUI)
else(ICUB_USE_GtkMM)
	message(STATUS "GtkMM not selected, skipping frameGrabberGui2")
	message(STATUS "GtkMM not selected, skipping focusTool")
	message(STATUS "GtkMM not selected, skipping iCubInterfaceGui")
endif(ICUB_USE_GtkMM)

if(ICUB_USE_Qt3 AND ICUB_USE_OpenGL AND ICUB_USE_GLUT)
	add_subdirectory(iCubGui)
else(ICUB_USE_Qt3 AND ICUB_USE_OpenGL AND ICUB_USE_GLUT)
	message(STATUS "Qt3 or OpenGL or GLUT not selected, skipping iCubGui")
endif(ICUB_USE_Qt3 AND ICUB_USE_OpenGL AND ICUB_USE_GLUT)


add_subdirectory(controlBoardDumper)
add_subdirectory(dataDumper)
add_subdirectory(simpleClient)
add_subdirectory(testStereoMatch)

#canLoader needs GtkPlus, but it contains a library needed by other modules
add_subdirectory(canLoader)

if(ICUB_USE_GtkPlus)
	add_subdirectory(iCubSkinGui)
	add_subdirectory(robotMotorGui)
else(ICUB_USE_GtkPlus)
	message(STATUS "GtkPlus not selected, skipping iCubSkinGui")
    message(STATUS "GtkPlus not selected, skipping robotMotorGui")
endif(ICUB_USE_GtkPlus)

if(ICUB_USE_OpenCV)
	add_subdirectory(laserScannerGui)
else(ICUB_USE_OpenCV)
	message(STATUS "OpenCV not selected, skipping laserScannerGui")
endif(ICUB_USE_OpenCV)

if(ICUB_USE_SDL)
	add_subdirectory(joystickCtrl)
else(ICUB_USE_SDL)
	message(STATUS "SDL not selected, skipping joystickCtrl")
endif(ICUB_USE_SDL)

if(ICUB_USE_GtkMM)
	add_subdirectory(frameGrabberGui2)
else(ICUB_USE_GtkMM)
	message(STATUS "GtkMM not selected, skipping frameGrabberGui2")
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



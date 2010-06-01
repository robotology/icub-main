add_subdirectory(controlBoardDumper)
add_subdirectory(dataDumper)
add_subdirectory(testStereoMatch)
add_subdirectory(velocityObserver)

if(ICUB_USE_GtkPlus)
	add_subdirectory(canLoader)
else(ICUB_USE_GtkPlus)
	message(status "GtkPlus not selected, skipping canLoader")
endif(ICUB_USE_GtkPlus)

if(ICUB_USE_OpenCV)
	add_subdirectory(indepMotionDetector)
else(ICUB_USE_OpenCV)
	message(status "OpenCV not selected, skipping indepMotionDetector")
endif(ICUB_USE_OpenCV)

set(BUILD_MATLAB_BASED_TOOLS FALSE CACHE BOOL "Dou you want to build MATLAB-based tools?")
if(BUILD_MATLAB_BASED_TOOLS)
	add_subdirectory(iKinArmView)
    add_subdirectory(iKinGazeView)
    add_subdirectory(signalScope)
endif(BUILD_MATLAB_BASED_TOOLS)


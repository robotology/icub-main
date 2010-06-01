add_subdirectory(controlBoardDumper)
add_subdirectory(dataDumper)
add_subdirectory(simpleClient)
add_subdirectory(testStereoMatch)
add_subdirectory(velocityControl)
add_subdirectory(velocityObserver)

if(ICUB_USE_GtkPlus)
	add_subdirectory(canLoader)
else(ICUB_USE_GtkPlus)
	message(STATUS "GtkPlus not selected, skipping canLoader")
endif(ICUB_USE_GtkPlus)

if(ICUB_USE_OpenCV)
	add_subdirectory(indepMotionDetector)
else(ICUB_USE_OpenCV)
	message(STATUS "OpenCV not found/selected, skipping indepMotionDetector")
endif(ICUB_USE_OpenCV)

set(BUILD_MATLAB_BASED_TOOLS FALSE CACHE BOOL "Dou you want to build MATLAB-based tools?")
if(BUILD_MATLAB_BASED_TOOLS)
    find_path(MATLAB_INCLUDE_DIRS engine.h NO_DEFAULT_PATH)
    if(WIN32)
       find_path(MATLAB_LIBRARIES_DIRS libeng.lib NO_DEFAULT_PATH)
    else(WIN32)
       FIND_PATH(MATLAB_LIBRARIES_DIRS eng NO_DEFAULT_PATH)
    endif(WIN32)
    
    if(WIN32)
       set(MATLAB_LIBRARIES ${MATLAB_LIBRARIES_DIRS}/libeng.lib
                            ${MATLAB_LIBRARIES_DIRS}/libmx.lib
                            ${MATLAB_LIBRARIES_DIRS}/libmex.lib
                            ${MATLAB_LIBRARIES_DIRS}/libmat.lib)
    else(WIN32)
       set(MATLAB_LIBRARIES ${MATLAB_LIBRARIES_DIRS}/libeng.so
                            ${MATLAB_LIBRARIES_DIRS}/libmx.so
                            ${MATLAB_LIBRARIES_DIRS}/libmex.so
                            ${MATLAB_LIBRARIES_DIRS}/libmat.so)
    endif(WIN32)

	add_subdirectory(iKinArmView)
    add_subdirectory(iKinGazeView)
    add_subdirectory(signalScope)
else(BUILD_MATLAB_BASED_TOOLS)
    message(STATUS "skipping iKinArmView")
    message(STATUS "skipping iKinGazeView")
    message(STATUS "skipping signalScope")
endif(BUILD_MATLAB_BASED_TOOLS)


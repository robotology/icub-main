PROJECT (iCub)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.


####
# Setup app directory, this is a target called "app" that is added to the main build.
# first set up some files in the application directory
icub_app(app)
set(app-files "${CMAKE_CURRENT_SOURCE_DIR}/ICUB_ROOT.ini;${CMAKE_CURRENT_SOURCE_DIR}/icubapp.py;${CMAKE_CURRENT_SOURCE_DIR}/app.txt.template")
			  
icub_app_install(app
				ROOT
				FILES ${app-files})

#################################

### Go through single applications

add_subdirectory(default)
add_subdirectory(actionPrimitivesExample)
add_subdirectory(cameraCalibration)
add_subdirectory(cartesianSolver)
add_subdirectory(demoGrasp_IIT_ISR)
add_subdirectory(iKinView)
add_subdirectory(faceExpressions)
add_subdirectory(faceTrackerDemo)
add_subdirectory(graspDetector)
add_subdirectory(objectsPropertiesCollector)
add_subdirectory(pf3dTracker)
add_subdirectory(signalScope)
add_subdirectory(simCartesianControl)
add_subdirectory(demoy3)
add_subdirectory(robotScripting)
add_subdirectory(iCubGui)
add_subdirectory(robots)



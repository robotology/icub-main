# Copyright: 2010 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

PROJECT (iCub)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.


####
# Setup app directory, this is a target called "app" that is added to the main build.
# first set up some files in the application directory
icub_app(app)

## policy file go to the icub application destination directory
icub_app_install(app
				ROOT
				FILES "${CMAKE_CURRENT_SOURCE_DIR}/ICUB_ROOT.ini")

## some other files go inside the "application directory"/app 
icub_app_install(app
				ROOT
				FILES "${CMAKE_CURRENT_SOURCE_DIR}/ymanager.ini;${CMAKE_CURRENT_SOURCE_DIR}/icubapp.py;${CMAKE_CURRENT_SOURCE_DIR}/app.txt.template"
				DESTINATION app
                FORCE_UPDATE
				)
#################################

### Go through single applications

add_subdirectory(default)
add_subdirectory(armCartesianController)
add_subdirectory(cameraCalibration)
add_subdirectory(cartesianSolver)
add_subdirectory(ctpService)
add_subdirectory(demoy3)
add_subdirectory(demoForceControl)
add_subdirectory(demoGrasp_IIT_ISR)
add_subdirectory(demoLogpolarGrabber)
add_subdirectory(controlBoardDumper)
add_subdirectory(iCubGui)
add_subdirectory(iCubSkinDemo)
add_subdirectory(faceExpressions)
add_subdirectory(logpolarRemapper)
add_subdirectory(objectsPropertiesCollector)
add_subdirectory(pf3dTracker)
#add_subdirectory(pf3dBottomup)
add_subdirectory(simCartesianControl)
add_subdirectory(robotScripting)
add_subdirectory(skinGui)
add_subdirectory(wholeBodyTorqueObserver)
add_subdirectory(robots)
add_subdirectory(simConfig)
add_subdirectory(iCubTest)
add_subdirectory(iCubCluster)
add_subdirectory(joystickCtrl)
add_subdirectory(iCubInterfaceGui)

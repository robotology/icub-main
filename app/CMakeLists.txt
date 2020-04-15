# Copyright: 2010 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

PROJECT (iCub)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

### Go through single applications

add_subdirectory(default)
add_subdirectory(actionsRenderingEngine)
add_subdirectory(cameraCalibration)
add_subdirectory(cartesianSolver)
add_subdirectory(ctpService)
add_subdirectory(controlBoardDumper)
add_subdirectory(iCubGui)
add_subdirectory(iKinGazeCtrl)
add_subdirectory(faceExpressions)
add_subdirectory(objectsPropertiesCollector)
add_subdirectory(simCartesianControl)
add_subdirectory(robotScripting)
add_subdirectory(skinGui)
add_subdirectory(wholeBodyDynamics)
add_subdirectory(simConfig)
add_subdirectory(yarpmanager)
add_subdirectory(joystickCtrl)
add_subdirectory(iCubStartup)
add_subdirectory(simFaceExpressions)
add_subdirectory(fingersTuner)
add_subdirectory(depth2kin)
add_subdirectory(fingertipsPortScope)
add_subdirectory(oculus)
add_subdirectory(wholeBodyPlayer)

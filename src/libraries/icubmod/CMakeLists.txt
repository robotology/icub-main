PROJECT(iCubModules)

# Prepare a device library called "icubmod".
# A test executable called "icubmoddev" will also be produced.

include(${YARP_DIR}/conf/YarpPlugin.cmake)

project(icubmod)

BEGIN_PLUGIN_LIBRARY(icubmod)
  add_subdirectory(armcalibrator)
  add_subdirectory(canBusMotionControl)
  add_subdirectory(cartesianController)
  add_subdirectory(cfw2Can)
  add_subdirectory(dragonfly2)
  add_subdirectory(esdCan)
  add_subdirectory(esdSniffer)
  add_subdirectory(fakeCan)
  add_subdirectory(gazeController)
  add_subdirectory(handcalibrator)
  add_subdirectory(headcalibrator)
  add_subdirectory(legscalibrator)
  add_subdirectory(logpolarClient)
  add_subdirectory(logpolarGrabber)
  add_subdirectory(motionControlLib)
  add_subdirectory(plxCan)
  add_subdirectory(staticgrabber)
  add_subdirectory(torsoonlycalibrator)
  add_subdirectory(xsensmtx)
  add_subdirectory(skinprototype)
END_PLUGIN_LIBRARY(icubmod)

icub_export_library(icubmod)

ADD_PLUGIN_LIBRARY_EXECUTABLE(icubmoddev icubmod)

INSTALL(TARGETS icubmoddev DESTINATION bin)
#INSTALL(TARGETS icubmod DESTINATION lib)

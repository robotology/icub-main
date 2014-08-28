# Copyright: (C) 2010-2013 RobotCub Consortium and iCub Facility Istituto Italiano di Tecnologia
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Prepare a device library called "icubmod".
# A test executable called "icubmoddev" will also be produced.

include(YarpPlugin)


#unset(ICUB_COMPILE_EMBOBJ_LIBRARY\)
set(ICUB_COMPILE_EMBOBJ_LIBRARY OFF CACHE INTERNAL "use the embObjLib lib" FORCE)

project(icubmod)

add_subdirectory(debugStream)

option (ICUB_EOPROT_MN_VERSION_1_0 "use OFF unless you want test old mode" OFF)
mark_as_advanced (ICUB_EOPROT_MN_VERSION_1_0)


if(ICUB_EOPROT_MN_VERSION_1_0)
add_definitions(-DEOPROT_USE_MN_VERSION_1_0)
else(ICUB_EOPROT_MN_VERSION_1_0)
add_definitions(-DEOPROT_USE_MN_VERSION_2_0)
endif(ICUB_EOPROT_MN_VERSION_1_0)

BEGIN_PLUGIN_LIBRARY(icubmod)
  add_subdirectory(armcalibrator)
  add_subdirectory(canBusMotionControl)
  add_subdirectory(cartesianController)
  add_subdirectory(cfw2Can)
  add_subdirectory(dragonfly2)
  add_subdirectory(esdCan)
  add_subdirectory(sharedCan)
  add_subdirectory(socketCan)
  add_subdirectory(esdSniffer)
  add_subdirectory(fakeCan)
  add_subdirectory(gazeController)
  add_subdirectory(handcalibrator)
  add_subdirectory(parametricCalibrator)
  add_subdirectory(parametricCalibratorEth)
  add_subdirectory(legscalibrator)
  add_subdirectory(logpolarClient)
  add_subdirectory(logpolarGrabber)
  add_subdirectory(motionControlLib)
  add_subdirectory(plxCan)
  add_subdirectory(staticgrabber)
  add_subdirectory(torsoonlycalibrator)
  add_subdirectory(xsensmtx)
  add_subdirectory(debugInterfaceClient)
  add_subdirectory(debugInterfaceWrapper)
  add_subdirectory(skinprototype)
  add_subdirectory(analogServer)
  add_subdirectory(skinWrapper)
  add_subdirectory(canBusAnalogSensor)
  add_subdirectory(canBusVirtualAnalogSensor)
  add_subdirectory(canBusInertialMTB)
  add_subdirectory(canBusSkin)
  add_subdirectory(embObjMotionControl)
  add_subdirectory(embObjSkin)
  add_subdirectory(embObjAnalog)
  add_subdirectory(embObjVirtualAnalogSensor)
if (ICUB_ICUBINTERFACE_EXPERIMENTAL)
  add_subdirectory(coman)
  add_subdirectory(imu3DM_GX3)
endif()
END_PLUGIN_LIBRARY(icubmod)

if(ICUB_COMPILE_COMAN_LIBRARY )
# adding subdirectory for common library used by coman devices, if anyone of them is selected
  add_subdirectory(coman/comanLib)
endif(ICUB_COMPILE_COMAN_LIBRARY )

if(ICUB_COMPILE_EMBOBJ_LIBRARY)
# adding subdirectory for embObj library used by EMS devices, if anyone of them is selected
  add_subdirectory(embObjLib)
endif(ICUB_COMPILE_EMBOBJ_LIBRARY)

icub_export_library(icubmod)

ADD_PLUGIN_LIBRARY_EXECUTABLE(icubmoddev icubmod)
target_link_libraries(icubmoddev YARP::YARP_OS YARP::YARP_dev YARP::YARP_init)
INSTALL(TARGETS icubmoddev DESTINATION bin)
#INSTALL(TARGETS icubmod DESTINATION lib)


# Copyright: (C) 2010-2013 RobotCub Consortium and iCub Facility Istituto Italiano di Tecnologia
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Prepare a device library called "icubmod".
# A test executable called "icubmoddev" will also be produced.

#unset(ICUB_COMPILE_EMBOBJ_LIBRARY\)
set(ICUB_COMPILE_EMBOBJ_LIBRARY OFF CACHE INTERNAL "use the embObjLib lib" FORCE)

project(icubmod)

option (WIP_FEATURE_CHECK_PROTOCOL_VERSION "use OFF until this feature is tested well enough" OFF)
mark_as_advanced (WIP_FEATURE_CHECK_PROTOCOL_VERSION)

if(WIP_FEATURE_CHECK_PROTOCOL_VERSION)
  add_definitions(-D_WIP_CHECK_PROTOCOL_VERSION_)
endif()

# The committed versions of the param parsers is not compatible with YARP 3.12.0, so depending on the YARP version we set
# the option to automatically run the generator to ON if we detect an incompatible YARP version. This can be changed when
# compatibility with YARP 3.11 is dropped and YARP 3.12 compatible param parser are generated
if(YARP_VERSION VERSION_LESS 3.11.100)
  set(ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE OFF)
else()
  set(ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE ON)
endif()

option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Generate the param parser for icub-main devices" ${ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE})

yarp_begin_plugin_library(icubmod QUIET)
  # This variable is used to provide a private (not installed) header for backward compatibility of YARP interfaces
  # across different YARP versions
  set(YARP_COMPAT_SHIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yarpCompatShim)

  add_subdirectory(canBusMotionControl)
  add_subdirectory(cartesianController)
  add_subdirectory(cfw2Can)
  add_subdirectory(dragonfly2)
  add_subdirectory(esdCan)
  add_subdirectory(sharedCan)
  add_subdirectory(socketCan)
  add_subdirectory(esdSniffer)
  add_subdirectory(fakeCan)
  add_subdirectory(bcbBattery)
  add_subdirectory(bmsBattery)
  add_subdirectory(gazeController)
  add_subdirectory(parametricCalibrator)
  add_subdirectory(parametricCalibratorEth)
  add_subdirectory(motionControlLib)
  add_subdirectory(plxCan)
  add_subdirectory(xsensmtx)
  add_subdirectory(skinWrapper)
  add_subdirectory(canBusAnalogSensor)
  add_subdirectory(canBusFtSensor)
  add_subdirectory(canBusVirtualAnalogSensor)
  add_subdirectory(canBusInertialMTB)
  add_subdirectory(canBusSkin)
  add_subdirectory(embObjMotionControl)
  add_subdirectory(embObjSkin)
  add_subdirectory(embObjFTsensor)
  add_subdirectory(embObjMultipleFTsensors)
  add_subdirectory(embObjMais)
  add_subdirectory(embObjIMU)
  add_subdirectory(imuST_M1)
  add_subdirectory(imuFilter)
  add_subdirectory(embObjPOS)
  add_subdirectory(embObjBattery)
  add_subdirectory(rawValuesPublisherClient)
  add_subdirectory(rawValuesPublisherServer)
  add_subdirectory(fakeRawValuesPublisher)
  add_subdirectory(couplingICubHandMk2)
  add_subdirectory(couplingICubEye)
if (ICUB_ICUBINTERFACE_EXPERIMENTAL)
  add_subdirectory(imu3DM_GX3)
endif()
yarp_end_plugin_library(icubmod QUIET)

if(ICUB_COMPILE_EMBOBJ_LIBRARY)
# adding subdirectory for embObj library used by EMS devices, if anyone of them is selected
  add_subdirectory(embObjLib)
endif(ICUB_COMPILE_EMBOBJ_LIBRARY)

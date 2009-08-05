# Install script for directory: /home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/libroboop.a")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/bench.cpp")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/clik.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/config.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/control_select.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/controller.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/dynamics_sim.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/gnugraph.cpp")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/gnugraph.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/quaternion.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/robot.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/stewart.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/trajectory.h")
FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/paulfitz/cvs/iCub/src/armSimpleAnticollisionFilter/roboop/source/utils.h")

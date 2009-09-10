# Install script for directory: /home/janine/Travail/libraries/icub/iCub/src/crawling/CrawlGeneratorTorso

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/janine/Travail/libraries/icub/iCub")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  IF(EXISTS "$ENV{DESTDIR}/home/janine/Travail/libraries/icub/iCub/bin/CrawlGeneratorTorso")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/janine/Travail/libraries/icub/iCub/bin/CrawlGeneratorTorso"
         RPATH "")
  ENDIF(EXISTS "$ENV{DESTDIR}/home/janine/Travail/libraries/icub/iCub/bin/CrawlGeneratorTorso")
  FILE(INSTALL DESTINATION "/home/janine/Travail/libraries/icub/iCub/bin" TYPE EXECUTABLE FILES "/home/janine/Travail/libraries/icub/iCub/bin/CrawlGeneratorTorso")
  IF(EXISTS "$ENV{DESTDIR}/home/janine/Travail/libraries/icub/iCub/bin/CrawlGeneratorTorso")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/home/janine/Travail/libraries/icub/iCub/bin/CrawlGeneratorTorso")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/janine/Travail/libraries/icub/iCub/bin/CrawlGeneratorTorso")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}/home/janine/Travail/libraries/icub/iCub/bin/CrawlGeneratorTorso")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/janine/Travail/libraries/icub/iCub/src/crawling/CrawlGeneratorTorso/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/janine/Travail/libraries/icub/iCub/src/crawling/CrawlGeneratorTorso/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)

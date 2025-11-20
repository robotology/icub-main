# Copyright: (C) 2025 iCub Facility - Istituto Italiano di Tecnologia
# Authors: SATHISH KUMAR S
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# yafu - Yet_Another_Firmware_Updater
# version: 1.0 - 2025/11/17

project(yafu)

# make install locations available
include(GNUInstallDirs)

# Only build on Linux
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(STATUS "Building YAFU on Linux")
else()
  message(STATUS "YAFU disabled on non-Linux platform (${CMAKE_SYSTEM_NAME}); skipping build and install.")
  return()
endif()

# require icub-firmware-shared
if(NOT ICUB_USE_icub_firmware_shared)
  message(STATUS "icub-firmware-shared not selected, disabling yafu")
  return()
endif()

# sources
set(SRCS
    src/yafu.cpp
    src/yafu.h
)

# target
add_executable(${PROJECT_NAME} ${SRCS})

# include & link firmware-shared and other deps
target_link_libraries(${PROJECT_NAME} PRIVATE
    icub_firmware_shared::embobj
)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)

# Install binary and helper scripts
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(PROGRAMS
        yafu_discover.sh
        yafu_parallel.sh
        yafu_program.sh
        yafu_restart.sh
        DESTINATION ${CMAKE_INSTALL_BINDIR})

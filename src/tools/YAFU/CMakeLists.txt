# YAFU - Yet_Another_Firmware_Updater
# version: 1.0

project(YAFU)

# require icub-firmware-shared (guard like other tools)
if(NOT ICUB_USE_icub_firmware_shared)
  message(STATUS "icub-firmware-shared not selected, disabling YAFU")
  return()
endif()

# sources
set(SRCS
    src/test.cpp
    src/test.h
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

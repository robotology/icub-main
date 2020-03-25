
project(strainCalibLib)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../ethLoader/ethLoaderLib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../canLoader/canLoaderLib)
include_directories(${GTK2_INCLUDE_DIRS})


add_library(${PROJECT_NAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECT_NAME} YARP::YARP_os
                                      ACE::ACE
                                      icub_firmware_shared::embobj)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

icub_export_library(${PROJECT_NAME})








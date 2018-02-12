
set(PROJECTNAME strainCalibLib)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../ethLoader/ethLoaderLib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../canLoader/canLoaderLib)
include_directories(${ACE_INCLUDE_DIRS})
include_directories(${GTK2_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})

include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR}/canProtocolLib)
include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR}/)
include_directories(${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/plus/comm-v2/icub/)
include_directories(${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/core/core)


add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${ACE_LIBRARIES})

icub_export_library(strainCalibLib)








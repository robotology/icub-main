
set(PROJECTNAME strainCalibGUI)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../ethLoader/ethLoaderLib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../canLoader/canLoaderLib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../strainCalibLib)


include_directories(${ACE_INCLUDE_DIRS})
include_directories(${GTK2_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR})

include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR}/canProtocolLib)

include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR}/)
include_directories(${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/plus/comm-v2/icub/)
include_directories(${icub_firmware_shared_embobj_INCLUDE_DIR}/embobj/core/core)

# libsigc++ 2.5.1 or later requires c++11. This is automatically
# enabled with CMake 3.1 or later (using FindGTK2.cmake from CMake
# 3.5). With older versions, we enable it here.
if(NOT ${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_LESS 3.1)
  message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(CMAKE_VERSION VERSION_LESS 3.1)
  if(NOT ${GTK2_SIGC++_VERSION_MAJOR}.${GTK2_SIGC++_VERSION_MINOR}.${GTK2_SIGC++_VERSION_MICRO} VERSION_LESS 2.5.1)
    add_definitions(${CXX11_FLAGS})
  endif()
endif()

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME}
                      canLoaderLib
                      strainCalibLib
                      ${GTK2_LIBRARIES}
                      ${YARP_LIBRARIES}
                      ${ACE_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)







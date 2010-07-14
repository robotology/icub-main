# Headers
set(icubgui_MOC_HDR animationview.h qavimator.h settingsdialog.h)

file(GLOB_RECURSE icubgui_SRC *.cpp)
file(GLOB_RECURSE icubgui_HDR *.h)
source_group("Source Files" FILES ${icubgui_SRC})
source_group("Header Files" FILES ${icubgui_HDR})

qt_wrap_cpp(iCubGui icubgui_MOC_SRC ${icubgui_MOC_HDR})

# Include path to the generated header files
include_directories(${PROJECT_SOURCE_DIR}/src ${QTOPENGL_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

# Set defines
add_definitions(${QTOPENGL_DEFINITIONS})

# Compile and link
add_executable(iCubGui ${icubgui_HDR} ${icubgui_SRC} ${icubgui_MOC_SRC})
target_link_libraries(iCubGui ${QTOPENGL_LIBRARIES} ${YARP_LIBRARIES})

# Install iCubGui
install(TARGETS iCubGui RUNTIME DESTINATION bin)
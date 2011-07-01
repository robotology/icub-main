# Copyright (C) 2009 RobotCub Consortium
# Author: Alessandro Scalzo alessandro.scalzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Headers
set(icubgui_MOC_HDR animationview.h qavimator.h settingsdialog.h)

file(GLOB_RECURSE icubgui_SRC *.cpp)
file(GLOB_RECURSE icubgui_HDR *.h)
source_group("Source Files" FILES ${icubgui_SRC})
source_group("Header Files" FILES ${icubgui_HDR})

qt_wrap_cpp(iCubGui icubgui_MOC_SRC ${icubgui_MOC_HDR})

# Include path to the generated header files
include_directories(${PROJECT_SOURCE_DIR}/src ${Qt3_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

# Set defines
add_definitions(${QT_DEFINITIONS})

# Compile and link
add_executable(iCubGui ${icubgui_HDR} ${icubgui_SRC} ${icubgui_MOC_SRC})
target_link_libraries(iCubGui ${Qt3_LIBRARIES} ${GLUT_LIBRARIES} ${OpenGL_LIBRARIES} ${YARP_LIBRARIES})

# Install iCubGui
install(TARGETS iCubGui RUNTIME DESTINATION bin)
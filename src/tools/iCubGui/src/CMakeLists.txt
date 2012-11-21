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
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${GLUT_INCLUDE_DIRS}
    ${OpenGL_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${skinDynLib_INCLUDE_DIRS}
)

if(ICUB_USE_QT4_QT3_SUPPORT)
    include(${QT_USE_FILE})
    add_definitions("-DICUB_USE_QT4_QT3_SUPPORT")
else(ICUB_USE_QT4_QT3_SUPPORT)
    include_directories(${Qt3_INCLUDE_DIRS})
endif(ICUB_USE_QT4_QT3_SUPPORT)

# Set defines
add_definitions(${QT_DEFINITIONS})

# Compile and link
add_executable(iCubGui ${icubgui_HDR} ${icubgui_SRC} ${icubgui_MOC_SRC})
target_link_libraries(iCubGui
    ${GLUT_LIBRARIES}
    ${OpenGL_LIBRARIES}
    ${YARP_LIBRARIES}
    skinDynLib
)

if(ICUB_USE_QT4_QT3_SUPPORT)
   target_link_libraries(iCubGui ${QT_QTCORE_LIBRARY}
                                 ${QT_QTGUI_LIBRARY}
                                 ${QT_QTOPENGL_LIBRARY}
                                 ${QT_QT3SUPPORT_LIBRARY})
else(ICUB_USE_QT4_QT3_SUPPORT)
    target_link_libraries(iCubGui ${Qt3_LIBRARIES})
endif(ICUB_USE_QT4_QT3_SUPPORT)

# Install iCubGui
install(TARGETS iCubGui RUNTIME DESTINATION bin)

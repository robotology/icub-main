# Sources
SET(icubgui_SRC animationview.cpp bvh.cpp bvhnode.cpp camera.cpp main.cpp qavimator.cpp settings.cpp settingsdialog.cpp)

# Headers
SET(icubgui_MOC_HDR animationview.h qavimator.h settingsdialog.h)
SET(icubgui_HDR bvh.h bvhnode.h camera.h mesh.h playstate.h settings.h ui_mainapplicationform.h ui_settingsdialogform.h ${icubgui_MOC_HDR})

# Interfaces
#SET(icubgui_UI mainapplicationform.ui settingsdialogform.ui)

# Prepare and generate all needed sources and headers
#qt_wrap_ui(icubgui icubgui_UI_HDR icubgui_UI_SRC ${icubgui_UI})
qt_wrap_cpp(iCubGui icubgui_MOC_SRC ${icubgui_MOC_HDR})

# Include path to the generated header files
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)

# Set defines
ADD_DEFINITIONS(${QT_DEFINITIONS})

# Compile and link
ADD_EXECUTABLE (iCubGui ${icubgui_HDR} ${icubgui_SRC} ${icubgui_MOC_SRC})
TARGET_LINK_LIBRARIES (iCubGui ${QT_QTOPENGL_LIBRARY} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

# Install iCubGui
INSTALL (TARGETS iCubGui RUNTIME DESTINATION bin)
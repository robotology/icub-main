PROJECT(iCubModules)

# Prepare a device library called "icubmod".
# A test executable called "icubmoddev" will also be produced.

include(${YARP_DIR}/conf/YarpPlugin.cmake)

BEGIN_PLUGIN_LIBRARY(icubmod)

  add_subdirectory(staticgrabber)

END_PLUGIN_LIBRARY(icubmod)

YARP_PREPARE_PLUGINS()

IF(MSVC)
    SET_TARGET_PROPERTIES(icubmoddev PROPERTIES COMPILE_FLAGS /wd4996)
    SET_TARGET_PROPERTIES(icubmod PROPERTIES COMPILE_FLAGS /wd4996)
ENDIF(MSVC)

INSTALL(TARGETS icubmoddev DESTINATION bin)
INSTALL(TARGETS icubmod DESTINATION lib)


PROJECT(iCubModules)

# Prepare a device library called "icubmod".
# A test executable called "icubmoddev" will also be produced.

include(${YARP_DIR}/conf/YarpPlugin.cmake)

project(icubmod)

BEGIN_PLUGIN_LIBRARY(icubmod)
  add_subdirectory(staticgrabber)
  add_subdirectory(plxCan)
END_PLUGIN_LIBRARY(icubmod)

#INSTALL(TARGETS icubmoddev DESTINATION bin)
#INSTALL(TARGETS icubmod DESTINATION lib)

ADD_PLUGIN_LIBRARY_EXECUTABLE(icubmoddev icubmod)


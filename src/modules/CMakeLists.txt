PROJECT(iCubModules)

FIND_PACKAGE(YARP REQUIRED)

# Prepare a device library called "icubmod".
# A test executable called "icubmoddev" will also be produced.

BEGIN_DEVICE_LIBRARY(icubmod)
  add_subdirectory(staticgrabber)
END_DEVICE_LIBRARY(icubmod)

IF(MSVC)
    SET_TARGET_PROPERTIES(icubmoddev PROPERTIES COMPILE_FLAGS /wd4996)
    SET_TARGET_PROPERTIES(icubmod PROPERTIES COMPILE_FLAGS /wd4996)
ENDIF(MSVC)

#add_subdirectory(motionControlLib)
IF (USE_ICUB_MOD)
  INSTALL(TARGETS icubmoddev DESTINATION bin)
  INSTALL(TARGETS icubmod DESTINATION lib)
ENDIF (USE_ICUB_MOD)

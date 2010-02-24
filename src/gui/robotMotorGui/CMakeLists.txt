PROJECT(robotMotorGui)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(CTRLLIB REQUIRED)
INCLUDE_DIRECTORIES(${CTRLLIB_INCLUDE_DIRS})

# check if we have already found GTK
IF(NOT GtkPlus_FOUND)
  FIND_PACKAGE(GtkPlus)
ENDIF (NOT GtkPlus_FOUND)

IF (GtkPlus_FOUND)
  ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
  LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS} ${CTRLLIB_LIBRARIES})
  INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})

  if(USE_ICUB_MOD)
    add_definitions(-DUSE_ICUB_MOD)
    find_package(icubmod)
    import_devices(${ICUB_DIR}/src/gui/robotMotorGui/src/include/drivers.h icubmod)
    FILE(GLOB_RECURSE folder_header src/include/*.cpp)
  else(USE_ICUB_MOD)
    if(NESTED_BUILD)
      set(folder_header src/include)
    else(NESTED_BUILD)
      add_definitions(-DUSE_ICUB_MOD)
      find_package(icubmod)
      import_devices(${ICUB_DIR}/src/gui/robotMotorGui/src/include/drivers.h icubmod)
      FILE(GLOB_RECURSE folder_header src/include/*.cpp)
    endif(NESTED_BUILD)
  endif(USE_ICUB_MOD)

  FILE(GLOB_RECURSE folder_source src/*.cpp)
  SOURCE_GROUP("Source Files" FILES ${folder_source})

  ADD_EXECUTABLE(robotMotorGui ${folder_source} ${folder_header})

  INSTALL(TARGETS robotMotorGui RUNTIME DESTINATION bin)

  # organization of configuration files in installations
  SET(appdir app/default/conf)

  # configuration files
  INSTALL(FILES robotMotorGui.ini DESTINATION ${appdir})
ELSEIF (GtkPlux_FOUND)
  MESSAGE("gtk+ not found, won't compile robotMotorGui")
ENDIF (GtkPlus_FOUND)
#
#

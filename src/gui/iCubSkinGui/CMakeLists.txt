SET(KEYWORD "iCubSkinGui")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)

SET(USE_ICUB_MOD TRUE CACHE BOOL "Use separated icub_mod lib")

#---------------------------------------
IF(NOT GtkPlus_FOUND)
  FIND_PACKAGE(GtkPlus)
#  FIND_PACKAGE(GTHREAD)
ENDIF (NOT GtkPlus_FOUND)

IF (GtkPlus_FOUND)
  ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
  LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})
  INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})

  #---------------------------------------

  #set up gthread
  #ADD_DEFINITIONS(${GTHREAD_C_FLAGS})
  #LINK_LIBRARIES(${GTHREAD_LINK_FLAGS})
  #INCLUDE_DIRECTORIES(${GTHREAD_INCLUDE_DIR})


  #search for includes
  INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include        
    )

  IF(USE_ICUB_MOD)
    ADD_DEFINITIONS(-DUSE_ICUB_MOD)
    FIND_PACKAGE(icubmod)
    IMPORT_DEVICES("${ICUB_DIR}/src/gui/iCubSkinGui/include/iCub/drivers.h" icubmod)
  ENDIF (USE_ICUB_MOD)

  # Search for source code.
  FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
  FILE(GLOB folder_header include/iCub/*.h)
  SOURCE_GROUP("Source Files" FILES ${folder_source})
  SOURCE_GROUP("Header Files" FILES ${folder_header})


  LINK_DIRECTORIES(
    ${ICUB_SOURCE_DIR}
    )

  LINK_LIBRARIES(
    ${GTHREAD_LIBRARIES}	 	
    )

  # Set up our main executable.
  IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
      ${folder_source} 
      ${folder_header}
      )	
  ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
  ENDIF (folder_source)

  INSTALL_TARGETS(/bin ${KEYWORD})

ELSE (GtkPlus_Found)
  MESSAGE(STATUS "Gtk not found, Skipping iCubSkinGui")
ENDIF (GtkPlus_FOUND)

# Copyright: (C) 2009 RobotCub Consortium
# Authors: Giorgio Metta, Lorenzo Natale, Stephen Hart
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Creates:
# GTKMM_INCLUDE_DIR   - Directories to include to use GTKMM
# GTKMM_LINK_FLAGS    - Files to link against to use GTKMM
# GTKMM_LINK_DIR      - Directories containing libraries to use GTKMM
# GtkMM_FOUND         - If false, don't try to use GTKMM

# gtkmm and libglademm seem to be coupled in FindGtkMMWin32.cmake
# so mirror that behavior here

FIND_PACKAGE(PkgConfig)

IF(PKG_CONFIG_FOUND)
  PKG_CHECK_MODULES(GTKMM gtkmm-2.4)
  PKG_CHECK_MODULES(GLADE libglademm-2.4)
  PKG_CHECK_MODULES(GTHREAD libglademm-2.4)
ENDIF(PKG_CONFIG_FOUND)

IF (GTKMM_FOUND)
  MESSAGE(STATUS " pkg-config found gtkmm")
  SET(GtkMM_FOUND TRUE)
  SET(GtkMM_INCLUDE_DIRS ${GTKMM_INCLUDE_DIRS})
  SET(GtkMM_LIBRARY_DIRS ${GTKMM_LIBDIR})
  SET(GtkMM_LIBRARIES  ${GTKMM_LIBRARIES})
  SET(GtkMM_C_FLAGS  ${GTKMM_CFLAGS})
ELSE (GTKMM_FOUND)
  SET(GtkMM_FOUND FALSE)
  MESSAGE(STATUS " pkg-config could not find gtkmm")
ENDIF (GTKMM_FOUND)

IF (GLADE_FOUND)
  MESSAGE(STATUS " pkg-config found glade")
  IF (GTKMM_FOUND)
    SET(GtkMM_INCLUDE_DIRS ${GTKMM_INCLUDE_DIRS} ${GLADE_INCLUDE_DIRS})
    SET(GtkMM_LIBRARY_DIRS ${GTKMM_LIBDIR} ${GLADE_LIBDIR})
    SET(GtkMM_LIBRARIES  ${GTKMM_LIBRARIES} ${GLADE_LIBRARIES})
    SET(GtkMM_C_FLAGS  ${GTKMM_CFLAGS} ${GLADE_CFLAGS})
  ENDIF (GTKMM_FOUND)
ELSE (GLADE_FOUND)
  MESSAGE(STATUS " pkg-config could not find glade")
ENDIF (GLADE_FOUND)

IF (GTHREAD_FOUND)
  MESSAGE(STATUS " pkg-config found gthread")
  IF (GTKMM_FOUND)
    SET(GtkMM_INCLUDE_DIRS ${GTKMM_INCLUDE_DIRS} ${GTHREAD_INCLUDE_DIRS})
    SET(GtkMM_LIBRARY_DIRS ${GTKMM_LIBDIR} ${GTHREAD_LIBDIR})
    SET(GtkMM_LIBRARIES  ${GTKMM_LIBRARIES} ${GTHREAD_LIBRARIES})
    SET(GtkMM_C_FLAGS  ${GTKMM_CFLAGS} ${GTHREAD_CFLAGS})
  ENDIF (GTKMM_FOUND)
ELSE (GTHREAD_FOUND)
  MESSAGE(STATUS " pkg-config could not find gthread")
ENDIF (GTHREAD_FOUND)


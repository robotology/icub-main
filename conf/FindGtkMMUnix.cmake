# Copyright: (C) 2009 RobotCub Consortium
# Authors: Giorgio Metta, Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Creates:
# GTKMM_INCLUDE_DIR   - Directories to include to use GTKMM
# GTKMM_LINK_FLAGS    - Files to link against to use GTKMM
# GTKMM_LINK_DIR      - Directories containing libraries to use GTKMM
# GtkMM_FOUND         - If false, don't try to use GTKMM

# gtkmm and libglademm seem to be coupled in FindGtkMMWin32.cmake
# so mirror that behavior here

include(FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GTKMM gtkmm-2.4 libglademm-2.4 gthread-2.0)
  if (GTKMM_FOUND)
    set(GTKMM_INCLUDE_DIR ${GTKMM_INCLUDE_DIRS})
    set(GTKMM_LINK_DIR ${GTKMM_LIBRARY_DIRS})
    set(GTKMM_LINK_FLAGS ${GTKMM_LDFLAGS})
    set(GTKMM_C_FLAGS ${GTKMM_CFLAGS})
  endif (GTKMM_FOUND)
endif (PKG_CONFIG_FOUND)


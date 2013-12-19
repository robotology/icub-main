# Copyright : (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Created:
# ROBOLLI_INC_DIRS   - Include folders for robolli lib
# ROBOLLI_LIB        - Default library to link against
# ROBOLLI_FOUND      - If true, robolli was found

IF(NOT ROBOLLI_FOUND)
    if(UNIX)

    # CASE 1: library has been compiled and installed somewhere.
    #           ENV var COMAN_ROOT points to the source code (COMAN_shared folder)
    #           ENV var COMAN_DIR  points to the binaries, wherever they are
    find_path(   robolli_INCLUDE_DIRS
                 NAMES broadcast_data.h
                 PATHS $ENV{COMAN_DIR}
                 PATH_SUFFIXES include)

    if(NOT robolli_INCLUDE_DIRS)
        message(ERROR " robolli includes were not found into $ENV{COMAN_DIR} subfolders")
        set(ROBOLLI_FOUND false)
        return()
    endif(NOT robolli_INCLUDE_DIRS)

    find_library(robolli_LIB
                 NAMES robolli
                 PATHS $ENV{COMAN_DIR}
                 PATH_SUFFIXES lib)

    if(robolli_LIB)
        message(STATUS " found robolli" )
        set(ROBOLLI_FOUND true)
    else(robolli_LIB)
        message(ERROR " robolli library was not found into $ENV{COMAN_DIR} subfolders")
        set(ROBOLLI_FOUND false)
        return()
    endif(robolli_LIB)

    endif(UNIX)

    IF(WIN32)
        message(ERROR " Sorry, Robolli is not available for Windows right now. Disable coman devices!")
        set(ROBOLLI_FOUND FALSE force)
        SET(ROBOLLI_INC_DIRS)
        SET(ROBOLLI_LIB )
    ENDIF(WIN32)
    


ENDIF(NOT ROBOLLI_FOUND)





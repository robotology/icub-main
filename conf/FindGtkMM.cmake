# Copyright: (C) 2009 RobotCub Consortium
# Authors: Giorgio Metta, Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

IF(UNIX)
    INCLUDE(FindGtkMMUnix)
ELSE(UNIX)
    IF(WIN32 AND NOT CYGWIN)
        FIND_PACKAGE(GtkMMWin32)
    ENDIF(WIN32 AND NOT CYGWIN)
ENDIF(UNIX)

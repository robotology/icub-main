# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Created:
# CFW2CANAPI_INC_DIRS   - Directories to include to use esdcan api
# CFW2CANAPI_LIB        - Default library to link against to use the esdcan API
# CFF2CANAPI_FOUND      - If false, don't try to use esdcan API

IF(NOT CFW2CANAPI_FOUND)
    SET(CFW2CANAPI_DIR $ENV{CFW2CANAPI_DIR} CACHE PATH "Path to CFW2CANAPI")
    SET(CFW2CANAPI_INC_DIRS ${CFW2CANAPI_DIR}/LinuxDriver/API)
    
    IF(WIN32)
       #sorry not available in windows
    ELSE(WIN32)  
       FIND_LIBRARY(CFW2CANAPI_LIB cfw002 ${CFW2CANAPI_DIR}/LinuxDriver/API)
    ENDIF(WIN32)
    
    IF(CFW2CANAPI_LIB)
       SET(CFW2CANAPI_FOUND TRUE)
    ELSE(CFW2CANAPI_LIB)
       SET(CFW2CANAPI_FOUND FALSE)
       SET(CFW2CANAPI_INC_DIRS)
       SET(CFW2CANAPI_LIB )
    ENDIF(CFW2CANAPI_LIB)

ENDIF(NOT CFW2CANAPI_FOUND)





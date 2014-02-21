# Copyright: (C) 2009 RobotCub Consortium
# Authors: Alexandre Bernardino, Paul Fitzpatrick, Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Created:
# ESDCANAPI_INC_DIRS   - Directories to include to use esdcan api
# ESDCANAPI_LIB        - Default library to link against to use the esdcan API
# ESDCANAPI_FOUND      - If false, don't try to use esdcan API

IF(NOT ESDCANAPI_FOUND)
    # ###########################################################################
	#we look for the lib, give priority to ESDCANAPI_DIR which should point to 
	#the correct place, but also look in other locations. CAnSdkDir should be set
	#by the installer.  
    SET(ESDCANAPI_DIR $ENV{ESDCANAPI_DIR} CACHE PATH "Path to ESDCANAPI")
    
    IF(WIN32)
        # Add possible search paths
        LIST(APPEND ESDCAN_LIB_DIRS ${ESDCANAPI_DIR}/develop/vc)
        LIST(APPEND ESDCAN_LIB_DIRS ${CanSdkDir}/develop/vc)
        LIST(APPEND ESDCAN_LIB_DIRS ${ESDCANAPI_DIR}/winnt)
        
        LIST(APPEND ESDCAN_INC_DIRS ${ESDCANAPI_DIR})
        LIST(APPEND ESDCAN_INC_DIRS ${CanSdkDir})
        
        # Build directory names depending on system architecture
        IF(DEFINED ENV{PROGRAMFILES})
            LIST(APPEND ESDCAN_LIB_DIRS "$ENV{PROGRAMFILES}/CAN/develop/vc")
            LIST(APPEND ESDCAN_LIB_DIRS "$ENV{PROGRAMFILES}/ESD/CAN/SDK/lib/VC/i386")
            
            LIST(APPEND ESDCAN_INC_DIRS "$ENV{PROGRAMFILES}/CAN")
            LIST(APPEND ESDCAN_INC_DIRS "$ENV{PROGRAMFILES}/ESD/CAN/SDK")
        ENDIF(DEFINED ENV{PROGRAMFILES})
        IF(DEFINED ENV{ProgramW6432})
            LIST(APPEND ESDCAN_LIB_DIRS "$ENV{ProgramW6432}/CAN/develop/vc")
            LIST(APPEND ESDCAN_LIB_DIRS "$ENV{ProgramW6432}/ESD/CAN/SDK/lib/VC/i386")
            
            LIST(APPEND ESDCAN_INC_DIRS "$ENV{ProgramW6432}/CAN")
            LIST(APPEND ESDCAN_INC_DIRS "$ENV{ProgramW6432}/ESD/CAN/SDK")
        ENDIF(DEFINED ENV{ProgramW6432})
        
        # Find library        
        FIND_LIBRARY(   ESDCANAPI_LIB 
                        NAMES "ntcan"
                        PATHS ${ESDCAN_LIB_DIRS}
                        NO_DEFAULT_PATH)

        # Find include file
        FIND_PATH(  ESDCANAPI_INC_DIRS 
                    NAMES "ntcan.h "
                    PATHS ${ESDCAN_INC_DIRS}
                    PATH_SUFFIXES "include")
        ELSE(WIN32)  
           FIND_LIBRARY(ESDCANAPI_LIB ntcan 
                        ${ESDCANAPI_DIR}/lib32 
                        ${ESDCANAPI_DIR}/lib64)    
           SET(ESDCANAPI_INC_DIRS ${ESDCANAPI_DIR}/lib32)
    ENDIF(WIN32)
    
      # Handle the QUIETLY and REQUIRED arguments and set ESDCANAPI_FOUND to TRUE if all listed variables are TRUE
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(ESDCANAPI  DEFAULT_MSG
                                  ESDCANAPI_LIB ESDCANAPI_INC_DIRS)
    # ###########################################################################

    
    # ###########################################################################
    # Mark as done
    mark_as_advanced(ESDCANAPI_LIB ESDCANAPI_INC_DIRS)
    # ###########################################################################
ENDIF(NOT ESDCANAPI_FOUND)


# Created:
# ESDCANAPI_INC_DIRS   - Directories to include to use esdcan api
# ESDCANAPI_LIB        - Default library to link against to use the esdcan API
# ESDCANAPI_FOUND      - If false, don't try to use esdcan API

IF(NOT ESDCANAPI_FOUND)
    SET(ESDCANAPI_DIR $ENV{ESDCANAPI_DIR} CACHE PATH "Path to ESDCANAPI")
    
    IF(WIN32)
	#we look for the lib, give priority to ESDCANAPI_DIR which should point to 
	#the correct place, but also look in other locations. CAnSdkDir should be set
	#by the installer.
	FIND_LIBRARY(ESDCANAPI_LIB ntcan ${ESDCANAPI_DIR}/develop/vc
						   ${CanSdkDir}/develop/vc
						   "C:/Program Files/CAN/develop/vc" 
						   ${ESDCANAPI_DIR}/winnt
						   NO_DEFAULT_PATH)

	FIND_PATH(ESDCANAPI_INC_DIRS ntcan.h ${ESDCANAPI_DIR}/include
						   ${CanSdkDir}/include
						   "C:/Program Files/CAN/include")
    ELSE(WIN32)  
       FIND_LIBRARY(ESDCANAPI_LIB ntcan ${ESDCANAPI_DIR}/lib32 ${ESDCANAPI_DIR}/lib64)    
       SET(ESDCANAPI_INC_DIRS ${ESDCANAPI_DIR}/lib32)
    ENDIF(WIN32)
    
    IF(ESDCANAPI_LIB)
       SET(ESDCANAPI_FOUND TRUE)
    ELSE(ESDCANAPI_LIB)
       SET(ESDCANAPI_FOUND FALSE)
       SET(ESDCANAPI_INC_DIRS)
       SET(ESDCANAPI_LIB )
    ENDIF(ESDCANAPI_LIB)

ENDIF(NOT ESDCANAPI_FOUND)





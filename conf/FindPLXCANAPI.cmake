# Created:
# PLXCANAPI_INC_DIRS   - Directories to include to use esdcan api
# PLXCANAPI_LIB        - Default library to link against to use the esdcan API
# PLXCANAPI_FOUND      - If false, don't try to use esdcan API

IF(NOT PLXCANAPI_FOUND)
    SET(PLXCANAPI_DIR $ENV{PLXCANAPI_DIR} CACHE PATH "Path to PLXCANAPI")
    SET(PLXCANAPI_INC_DIRS ${PLXCANAPI_DIR}/include)
    
    IF(WIN32)
       #sorry not available in windows
    ELSE(WIN32)  
       FIND_LIBRARY(PLXCANAPI_LIB plxcan ${PLXCANAPI_DIR}/lib NO_DEFAULT_PATH)    
    ENDIF(WIN32)
    
    IF(PLXCANAPI_LIB)
       SET(PLXCANAPI_FOUND TRUE)
    ELSE(PLXCANAPI_LIB)
       SET(PLXCANAPI_FOUND FALSE)
       SET(PLXCANAPI_INC_DIRS)
       SET(PLXCANAPI_LIB )
    ENDIF(PLXCANAPI_LIB)

ENDIF(NOT PLXCANAPI_FOUND)





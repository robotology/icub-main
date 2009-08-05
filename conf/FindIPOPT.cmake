# Created:
# IPOPT_INC_DIRS   - Directories to include to use IPOPT
# IPOPT_LIB        - Default library to link against to use IPOPT
# IPOPT_LIB_R      - Release library (for win) to link against to use IPOPT
# IPOPT_LIB_D      - Debug library (for win) to link against to use IPOPT
# IPOPT_FOUND      - If false, don't try to use IPOPT

IF(NOT IPOPT_FOUND)

    SET(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT main directory")
    SET(IPOPT_INC_DIRS ${IPOPT_DIR}/include/coin)
    
    IF(WIN32)
    
       SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:libcmt.lib;libcmtd.lib")

       FIND_LIBRARY(IPOPT_LIB_R libipopt  ${IPOPT_DIR}/lib NO_DEFAULT_PATH)
       FIND_LIBRARY(IPOPT_LIB_D libipoptD ${IPOPT_DIR}/lib NO_DEFAULT_PATH)
       SET(IPOPT_LIB optimized ${IPOPT_LIB_R} debug ${IPOPT_LIB_D})
    
    ELSE(WIN32)
    
       FIND_LIBRARY(IPOPT_LIB ipopt ${IPOPT_DIR}/lib NO_DEFAULT_PATH)
       IF(IPOPT_LIB)
          FILE(READ ${IPOPT_DIR}/share/doc/coin/Ipopt/ipopt_addlibs_cpp.txt IPOPT_DEP)
          STRING(REGEX REPLACE "-[^l][^ ]* " "" IPOPT_DEP ${IPOPT_DEP})
          STRING(REPLACE "-l" "" IPOPT_DEP ${IPOPT_DEP})
          SEPARATE_ARGUMENTS(IPOPT_DEP)
          SET(IPOPT_LIB   ${IPOPT_LIB} ${IPOPT_DEP})
          SET(IPOPT_LIB_R ${IPOPT_LIB})
          SET(IPOPT_LIB_D ${IPOPT_LIB})
       ENDIF(IPOPT_LIB)
    
    ENDIF(WIN32)
    
    IF(IPOPT_LIB)
       SET(IPOPT_FOUND TRUE)
    ELSE(IPOPT_LIB)
       SET(IPOPT_FOUND FALSE)
       SET(IPOPT_INC_DIRS)
       SET(IPOPT_LIB)
       SET(IPOPT_LIB_R)
       SET(IPOPT_LIB_D)
    ENDIF(IPOPT_LIB)

ENDIF(NOT IPOPT_FOUND)



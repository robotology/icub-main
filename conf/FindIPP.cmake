SET(VERBOSE OFF)

IF(VERBOSE)
MESSAGE(STATUS "Looking for IPP using FindIPP.cmake")
ENDIF(VERBOSE)

IF (NOT IPP_FOUND)

	#MESSAGE(STATUS "IPP Library")

	SET(LINK_LIB_HIGHGUI TRUE CACHE BOOL "Do you want to link against libhighgui?")
	IF (LINK_LIB_HIGHGUI AND WIN32)
  		SET(LINK_LIB_CVCAM)
	ENDIF(LINK_LIB_HIGHGUI AND WIN32)

	SET(IS_GNUCXX3 FALSE)
	SET(IS_GNUCXX4 FALSE)
	IF    (${CMAKE_COMPILER_IS_GNUCXX})

  		# MESSAGE(STATUS "Checking GNUCXX version 3/4 to determine  IPP /opt/net/ path")
  		EXEC_PROGRAM(${CMAKE_CXX_COMPILER} ARGS --version OUTPUT_VARIABLE CXX_COMPILER_VERSION)
  
	  IF   (CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
		 #   MESSAGE("DBG IPP for 3.x")
		 SET(IS_GNUCXX3 TRUE)
		 # ELSE (CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
		 #   MESSAGE("DBG not 3.x")
	  ENDIF(CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")

	  IF   (CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")
		 #   MESSAGE("DBG IPP for 4.x")
		 SET(IS_GNUCXX4 TRUE)
		 # ELSE (CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")
		 #   MESSAGE("DBG not 4.x")
	  ENDIF(CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")

	ENDIF (${CMAKE_COMPILER_IS_GNUCXX})

#----------- CHECK IPP_ROOT-----------------------------
IF (EXISTS "$ENV{IPP_ROOT}")
IF(VERBOSE)
MESSAGE("Exist IPP_ROOT. ")
ENDIF (VERBOSE)

SET(IPP_POSSIBLE_INCDIRS
  "$ENV{IPP_ROOT}"
  "$ENV{IPP_ROOT}/include"
	
  #"$ENV{IPP_ROOT}/include/cv" 
  #"$ENV{IPP_ROOT}/include/IPP" 
  #"$ENV{IPP_ROOT}/cxcore/include"
  #"$ENV{IPP_ROOT}/cv/include"
  #"$ENV{IPP_ROOT}/cvaux/include"
  #"$ENV{IPP_ROOT}/otherlibs/cvcam/include"
  #"$ENV{IPP_ROOT}/otherlibs/highgui/include"
  #"$ENV{IPP_ROOT}/otherlibs/highgui/"
  #"$ENV{IPP_ROOT}/cv/src"
)

SET(IPP_POSSIBLE_LIBRARY_PATHS
  "$ENV{IPP_ROOT}/lib"
  "$ENV{IPP_ROOT}/sharedlib"  
  "$ENV{IPP_ROOT}"
  /opt/intel/ipp/sharedlib
)
ENDIF (EXISTS "$ENV{IPP_ROOT}")
#--------------------------------------------------------

#-----------------CHECK IPP_DIR -----------------------
IF (EXISTS "$ENV{IPP_DIR}")
IF(VERBOSE)
MESSAGE("Exist IPP_DIR. ")
ENDIF(VERBOSE)

SET(IPP_POSSIBLE_INCDIRS
  "$ENV{IPP_DIR}"
  "$ENV{IPP_DIR}/include"

  #"$ENV{IPP_DIR}/include/cv" 
  #"$ENV{IPP_DIR}/include/IPP" 
  #"$ENV{IPP_DIR}/cxcore/include"
  #"$ENV{IPP_DIR}/cv/include"
  #"$ENV{IPP_DIR}/cvaux/include"
  #"$ENV{IPP_DIR}/otherlibs/cvcam/include"
  #"$ENV{IPP_DIR}/otherlibs/highgui/include"
  #"$ENV{IPP_DIR}/otherlibs/highgui/"
  #"$ENV{IPP_DIR}/cv/src"
)

SET(IPP_POSSIBLE_LIBRARY_PATHS
  "$ENV{IPP_DIR}"
  "$ENV{IPP_DIR}/sharedlib"
  "$ENV{IPP_DIR}/lib")

ENDIF(EXISTS "$ENV{IPP_DIR}")
#----------------------------------------------------------

# CHECK IPP_HOME
IF (EXISTS "$ENV{IPP_HOME}")
SET(IPP_POSSIBLE_INCDIRS
  "$ENV{IPP_HOME}"
  "$ENV{IPP_HOME}/include"
  #"$ENV{IPP_HOME}/include/cv"
  #"$ENV{IPP_HOME}/include/IPP"
)
ENDIF (EXISTS "$ENV{IPP_HOME}")

IF (NOT IPP_POSSIBLE_INCDIRS)
IF(VERBOSE)
MESSAGE('Not IPP_POSSIBLE INCDIRS')
ENDIF(VERBOSE)
SET(IPP_POSSIBLE_INCDIRS
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]"  
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]/include"
  "$ENV{ProgramFiles}/IPP"
  "$ENV{ProgramFiles}/IPP/include"
  #"$ENV{ProgramFiles}/IPP/cxcore/include"
  #"$ENV{ProgramFiles}/IPP/cv/include"
  #"$ENV{ProgramFiles}/IPP/cvaux/include"
  #"$ENV{ProgramFiles}/IPP/otherlibs/cvcam/include"
  #"$ENV{ProgramFiles}/IPP/otherlibs/highgui/include"
  #"$ENV{ProgramFiles}/IPP/otherlibs/highgui"
  #"$ENV{ProgramFiles}/IPP/cv/src"
  /usr/include/IPP
  /opt/intel/ipp/include
  /opt/intel/ipp/.1.1.042/em64t/include	
  /usr/local/include/IPP)
ENDIF (NOT IPP_POSSIBLE_INCDIRS)

IF (NOT IPP_POSSIBLE_LIBRARY_PATHS)
SET(IPP_POSSIBLE_LIBRARY_PATHS
  "$ENV{ProgramFiles}/IPP/lib"
  "/usr/local/lib"
  "/opt/intel/ipp/lib"
  "/opt/intel/ipp/sharedlib"
  "/usr/lib"
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]/lib")
ENDIF(NOT IPP_POSSIBLE_LIBRARY_PATHS)

IF   (IS_GNUCXX3)
  SET(IPP_POSSIBLE_INCDIRS ${IPP_POSSIBLE_INCDIRS} 
    /opt/net/gcc33/IPP/
    /opt/net/gcc33/IPP/include
    /opt/net/gcc33/IPP/include/IPP )
ENDIF(IS_GNUCXX3)
IF   (IS_GNUCXX4)
  SET(IPP_POSSIBLE_INCDIRS ${IPP_POSSIBLE_INCDIRS} 
    /opt/net/gcc41/IPP/
    /opt/net/gcc41/IPP/include
    /opt/net/gcc41/IPP/include/IPP )
ENDIF(IS_GNUCXX4)
#MESSAGE("DBG (IPP_POSSIBLE_INCDIRS=${IPP_POSSIBLE_INCDIRS}")

# candidates for IPP library directories:

IF   (IS_GNUCXX3)
  SET(IPP_POSSIBLE_LIBRARY_PATHS ${IPP_POSSIBLE_LIBRARY_PATHS}
    /opt/net/gcc33/IPP
    /opt/net/gcc33/IPP/lib )
ENDIF(IS_GNUCXX3)
IF   (IS_GNUCXX4)
  SET(IPP_POSSIBLE_LIBRARY_PATHS ${IPP_POSSIBLE_LIBRARY_PATHS}
    /opt/net/gcc41/IPP
    /opt/net/gcc41/IPP/lib
)
ENDIF(IS_GNUCXX4)

IF(VERBOSE)
MESSAGE("DBG (IPP_POSSIBLE_LIBRARY_PATHS=${IPP_POSSIBLE_LIBRARY_PATHS}")
ENDIF(VERBOSE)

# find (all) header files for include directories:
FIND_PATH(IPP_INCLUDE_DIR_MAIN   ipp.h  ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_AC   ippac.h  ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_CC    ippcc.h      ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_CH    ippch.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_CORE  ippcore.h ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_CV    ippcv.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_DC    ippdc.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_DEFS    ippdefs.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_I    ippi.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_J    ippj.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_M    ippm.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_R    ippr.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_S    ipps.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_SC    ippsc.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_SR    ippsr.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_VC    ippvc.h   ${IPP_POSSIBLE_INCDIRS} )
FIND_PATH(IPP_INCLUDE_DIR_VM    ippsvm.h   ${IPP_POSSIBLE_INCDIRS} )



#FIND_PATH(IPP_INCLUDE_DIR__CV    _cv.h   ${IPP_POSSIBLE_INCDIRS} )


#MESSAGE("DBG IPP_INCLUDE_DIR_CV=${IPP_INCLUDE_DIR_CV} ")

# find (all) libraries - some dont exist on Linux
FIND_LIBRARY(IPP_CC_LIBRARY
  NAMES ippcc 
  PATHS ${IPP_POSSIBLE_LIBRARY_PATHS}
  DOC "Location of the IPPcc")

FIND_LIBRARY(IPP_S_LIBRARY
  NAMES ipps 
  PATHS ${IPP_POSSIBLE_LIBRARY_PATHS}
  DOC "Location of the IPPs")

FIND_LIBRARY(IPP_CV_LIBRARY
  NAMES ippcv 
  PATHS ${IPP_POSSIBLE_LIBRARY_PATHS} 
  DOC "Location of the cvaux IPPcv")

FIND_LIBRARY(IPP_I_LIBRARY
  NAMES ippi
  PATHS ${IPP_POSSIBLE_LIBRARY_PATHS} 
  DOC "Location of the IPPi")

  

########
SET(IPP_FOUND ON)
########


###########################################
## CHECK HEADER FILES
# REQUIRED LIBS
FOREACH(INCDIR 
  IPP_INCLUDE_DIR_CV 
  IPP_INCLUDE_DIR_CC	
  IPP_INCLUDE_DIR_I
  IPP_INCLUDE_DIR_S
  IPP_INCLUDE_DIR_MAIN
)
  IF    (${INCDIR})
    SET(IPP_INCLUDE_DIRS ${IPP_INCLUDE_DIRS} ${${INCDIR}} )
	IF(VERBOSE)
	 MESSAGE("${IPP_INCLUDE_DIRS}  found for ${INCDIR}.keeping on IPP_FOUND ")
	ENDIF (VERBOSE)
  ELSE  (${INCDIR})
	IF(VERBOSE)
	 MESSAGE("${IPP_INCLUDE_DIRS} not found turning off IPP_FOUND")
	ENDIF(VERBOSE)
    SET(IPP_FOUND OFF)
  ENDIF (${INCDIR})  
ENDFOREACH(INCDIR)



# MESSAGE("DBG IPP_INCLUDE_DIR=${IPP_INCLUDE_DIR}")
# libcxcore does not seem to be always required (different distribution behave 
# differently)
#IF (IPP_INCLUDE_DIR_CXCORE)
#  SET(IPP_INCLUDE_DIRS ${IPP_INCLUDE_DIRS} ${IPP_INCLUDE_DIR_CXCORE})
#ELSE (IPP_INCLUDE_DIR_CXCORE)
#  IF (WIN32) #required in win
#	SET(IPP_FOUND OFF)
	#MESSAGE("- DBG IPP_INCLUDE_DIR_CXCORE=${IPP_INCLUDE_DIR_CXCORE} ")
#  ENDIF (WIN32)
#ENDIF(IPP_INCLUDE_DIR_CXCORE)


#IF(LINK_LIB_HIGHGUI)
#  IF (IPP_INCLUDE_DIR_HIGHGUI)
#	SET(IPP_INCLUDE_DIRS ${IPP_INCLUDE_DIRS} ${IPP_INCLUDE_DIR_HIGHGUI})
#  ELSE (IPP_INCLUDE_DIR_HIGHGUI)
#	IF (WIN32)
#	  SET(IPP_FOUND OFF)
	#MESSAGE("- DBG IPP_INCLUDE_DIR_HIGHGUI=${IPP_INCLUDE_DIR_HIGHGUI} ")
#	ENDIF (WIN32)
#  ENDIF(IPP_INCLUDE_DIR_HIGHGUI)
#ENDIF(LINK_LIB_HIGHGUI)

#################################


## LIBRARIES
# REQUIRED LIBRARIES
FOREACH(LIBNAME  
  IPP_I_LIBRARY 
  IPP_S_LIBRARY
  IPP_CC_LIBRARY
  IPP_CV_LIBRARY)
	IF (${LIBNAME})
    		SET(IPP_LIBRARIES ${IPP_LIBRARIES} ${${LIBNAME}} )
		IF(VERBOSE)
	    		MESSAGE("${LIBNAME} found for ${LIBNAME} .keeping on IPP_FOUND")
		ENDIF(VERBOSE)
  	ELSE  (${LIBNAME})
		IF(VERBOSE)
    			MESSAGE("${LIBNAME} not found turning off IPP_FOUND")
		ENDIF(VERBOSE)
    SET(IPP_FOUND OFF)
  	ENDIF (${LIBNAME})
ENDFOREACH(LIBNAME)

#IF (IPP_CXCORE_LIBRARY)
#  SET(IPP_LIBRARIES ${IPP_LIBRARIES} ${IPP_CXCORE_LIBRARY})
#ELSE (IPP_CXCORE_LIBRARY)
#  IF (WIN32) #this is required on windows
#	SET(IPP_FOUND OFF)
#	MESSAGE("IPP_CXCORE_LIBRARY not found turning off IPP_FOUND")
#  ENDIF (WIN32)
#ENDIF(IPP_CXCORE_LIBRARY)



#IF(LINK_LIB_HIGHGUI)
#  IF (IPP_HIGHGUI_LIBRARY)
#	SET(IPP_LIBRARIES ${IPP_LIBRARIES} ${IPP_HIGHGUI_LIBRARY})
#  ELSE (IPP_HIGHGUI_LIBRARY)
#	IF (WIN32) #this is required on windows
#	  SET(IPP_FOUND OFF)
#	  MESSAGE("IPP_HIGHGUI_LIBRARY not found turning off IPP_FOUND")
#	ENDIF (WIN32)
#  ENDIF(IPP_HIGHGUI_LIBRARY)
#ENDIF(LINK_LIB_HIGHGUI)

# get the link directory for rpath to be used with LINK_DIRECTORIES: 
IF (IPP_LIBRARY)
  GET_FILENAME_COMPONENT(IPP_LINK_DIRECTORIES ${IPP_LIBRARY} PATH)
ENDIF (IPP_LIBRARY)


# display help message
IF (NOT IPP_FOUND)
	IF(VERBOSE)
  		MESSAGE("IPP library or headers not found. "
  		"Please search manually or set env. variable IPP_ROOT to guide search." )
	ENDIF(VERBOSE)
ENDIF (NOT IPP_FOUND)

MARK_AS_ADVANCED(
  IPP_INCLUDE_DIRS  
  IPP_LIBRARIES
  IPP_LIBRARY
  #IPP_HIGHGUI_LIBRARY
  #IPP_CVAUX_LIBRARY
  #IPP_CXCORE_LIBRARY
  #IPP_CVCAM_LIBRARY
  IPP_DIR
)

ENDIF (NOT IPP_FOUND)

SET(VERBOSE OFF)

IF(VERBOSE)
	MESSAGE(STATUS "Looking for BML using FindBML.cmake")
ENDIF(VERBOSE)

IF (NOT BML_FOUND)

	#MESSAGE(STATUS "BML Library")

	SET(LINK_LIB_HIGHGUI TRUE CACHE BOOL "Do you want to link against libhighgui?")
	IF (LINK_LIB_HIGHGUI AND WIN32)
  		SET(LINK_LIB_CVCAM)
	ENDIF(LINK_LIB_HIGHGUI AND WIN32)

	SET(IS_GNUCXX3 FALSE)
	SET(IS_GNUCXX4 FALSE)
	IF    (${CMAKE_COMPILER_IS_GNUCXX})

  		# MESSAGE(STATUS "Checking GNUCXX version 3/4 to determine  BML /opt/net/ path")
  		EXEC_PROGRAM(${CMAKE_CXX_COMPILER} ARGS --version OUTPUT_VARIABLE CXX_COMPILER_VERSION)
  
	  IF   (CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
		 #   MESSAGE("DBG BML for 3.x")
		 SET(IS_GNUCXX3 TRUE)
		 # ELSE (CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
		 #   MESSAGE("DBG not 3.x")
	  ENDIF(CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")

	  IF   (CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")
		 #   MESSAGE("DBG BML for 4.x")
		 SET(IS_GNUCXX4 TRUE)
		 # ELSE (CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")
		 #   MESSAGE("DBG not 4.x")
	  ENDIF(CXX_COMPILER_VERSION MATCHES ".*4\\.[0-9].*")

	ENDIF (${CMAKE_COMPILER_IS_GNUCXX})

#----------- CHECK BML_ROOT-----------------------------
IF (EXISTS "$ENV{BML_ROOT}")
IF(VERBOSE)
MESSAGE("Exist BML_ROOT. ")
ENDIF (VERBOSE)

SET(BML_POSSIBLE_INCDIRS
  "$ENV{BML_ROOT}"
  "$ENV{BML_ROOT}/include"
  "$ENV{BML_ROOT}/include/iCub"
	
  #"$ENV{BML_ROOT}/include/cv" 
  #"$ENV{BML_ROOT}/include/BML" 
  #"$ENV{BML_ROOT}/cxcore/include"
  #"$ENV{BML_ROOT}/cv/include"
  #"$ENV{BML_ROOT}/cvaux/include"
  #"$ENV{BML_ROOT}/otherlibs/cvcam/include"
  #"$ENV{BML_ROOT}/otherlibs/highgui/include"
  #"$ENV{BML_ROOT}/otherlibs/highgui/"
  #"$ENV{BML_ROOT}/cv/src"
)

SET(BML_POSSIBLE_LIBRARY_PATHS
  "$ENV{BML_ROOT}/lib"
  "$ENV{BML_ROOT}/debug"
  "$ENV{BML_ROOT}/release"
  "$ENV{BML_ROOT}/sharedlib"  
  "$ENV{BML_ROOT}"
  /opt/intel/ipp/sharedlib
)
ENDIF (EXISTS "$ENV{BML_ROOT}")
#--------------------------------------------------------

#-----------------CHECK BML_DIR -----------------------
IF (EXISTS "$ENV{BML_DIR}")
IF(VERBOSE)
	MESSAGE("Exist BML_DIR. ")
ENDIF(VERBOSE)

SET(BML_POSSIBLE_INCDIRS
  "$ENV{BML_DIR}"
  "$ENV{BML_DIR}/include"
  "$ENV{BML_DIR}/include/iCub"

  #"$ENV{BML_DIR}/include/cv" 
  #"$ENV{BML_DIR}/include/BML" 
  #"$ENV{BML_DIR}/cxcore/include"
  #"$ENV{BML_DIR}/cv/include"
  #"$ENV{BML_DIR}/cvaux/include"
  #"$ENV{BML_DIR}/otherlibs/cvcam/include"
  #"$ENV{BML_DIR}/otherlibs/highgui/include"
  #"$ENV{BML_DIR}/otherlibs/highgui/"
  #"$ENV{BML_DIR}/cv/src"
)

SET(BML_POSSIBLE_LIBRARY_PATHS
  "$ENV{BML_DIR}"
  "$ENV{BML_DIR}/sharedlib"
  "$ENV{BML_DIR}/lib"
  "$ENV{BML_DIR}/debug"
  "$ENV{BML_DIR}/release"
)

ENDIF(EXISTS "$ENV{BML_DIR}")
#----------------------------------------------------------

# CHECK BML_HOME
IF (EXISTS "$ENV{BML_HOME}")
SET(BML_POSSIBLE_INCDIRS
  "$ENV{BML_HOME}"
  "$ENV{BML_HOME}/include"
  "$ENV{BML_HOME}/include/iCub"
  #"$ENV{BML_HOME}/include/cv"
  #"$ENV{BML_HOME}/include/BML"
)
ENDIF (EXISTS "$ENV{BML_HOME}")

IF (NOT BML_POSSIBLE_INCDIRS)
IF(VERBOSE)
MESSAGE('Not BML_POSSIBLE INCDIRS')
ENDIF(VERBOSE)
SET(BML_POSSIBLE_INCDIRS
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]"  
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]/include"
  "$ENV{ProgramFiles}/BML"
  "$ENV{ProgramFiles}/BML/include"
  #"$ENV{ProgramFiles}/BML/cxcore/include"
  #"$ENV{ProgramFiles}/BML/cv/include"
  #"$ENV{ProgramFiles}/BML/cvaux/include"
  #"$ENV{ProgramFiles}/BML/otherlibs/cvcam/include"
  #"$ENV{ProgramFiles}/BML/otherlibs/highgui/include"
  #"$ENV{ProgramFiles}/BML/otherlibs/highgui"
  #"$ENV{ProgramFiles}/BML/cv/src"
  /usr/include/BML
  /opt/intel/BML/include
  /opt/intel/BML/.1.1.042/em64t/include	
  /usr/local/include/BML)
ENDIF (NOT BML_POSSIBLE_INCDIRS)

IF (NOT BML_POSSIBLE_LIBRARY_PATHS)
SET(BML_POSSIBLE_LIBRARY_PATHS
  "$ENV{ProgramFiles}/BML/lib"
  "/usr/local/lib"
  "/opt/intel/BML/lib"
  "/opt/intel/BML/sharedlib"
  "/usr/lib"
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]/lib")
ENDIF(NOT BML_POSSIBLE_LIBRARY_PATHS)

IF   (IS_GNUCXX3)
  SET(BML_POSSIBLE_INCDIRS ${BML_POSSIBLE_INCDIRS} 
    /opt/net/gcc33/BML/
    /opt/net/gcc33/BML/include
    /opt/net/gcc33/BML/include/BML )
ENDIF(IS_GNUCXX3)
IF   (IS_GNUCXX4)
  SET(BML_POSSIBLE_INCDIRS ${BML_POSSIBLE_INCDIRS} 
    /opt/net/gcc41/BML/
    /opt/net/gcc41/BML/include
    /opt/net/gcc41/BML/include/BML )
ENDIF(IS_GNUCXX4)
#MESSAGE("DBG (BML_POSSIBLE_INCDIRS=${BML_POSSIBLE_INCDIRS}")

# candidates for BML library directories:

IF   (IS_GNUCXX3)
  SET(BML_POSSIBLE_LIBRARY_PATHS ${BML_POSSIBLE_LIBRARY_PATHS}
    /opt/net/gcc33/BML
    /opt/net/gcc33/BML/lib )
ENDIF(IS_GNUCXX3)
IF   (IS_GNUCXX4)
  SET(BML_POSSIBLE_LIBRARY_PATHS ${BML_POSSIBLE_LIBRARY_PATHS}
    /opt/net/gcc41/BML
    /opt/net/gcc41/BML/lib
)
ENDIF(IS_GNUCXX4)

IF(VERBOSE)
MESSAGE("DBG (BML_POSSIBLE_LIBRARY_PATHS=${BML_POSSIBLE_LIBRARY_PATHS}")
ENDIF(VERBOSE)

# find (all) header files for include directories:
FIND_PATH(BML_INCLUDE_DIR_CONNECTION  iCub/Connection.h  ${BML_POSSIBLE_INCDIRS} )
FIND_PATH(BML_INCLUDE_DIR_ELEMENT   iCub/Element.h  ${BML_POSSIBLE_INCDIRS} )
FIND_PATH(BML_INCLUDE_DIR_LAYER    iCub/Layer.h      ${BML_POSSIBLE_INCDIRS} )
FIND_PATH(BML_INCLUDE_DIR_MACHINEBOLTZMANN    iCub/MachineBoltzmann.h   ${BML_POSSIBLE_INCDIRS} )
FIND_PATH(BML_INCLUDE_DIR_ROW  iCub/Row.h ${BML_POSSIBLE_INCDIRS} )
FIND_PATH(BML_INCLUDE_DIR_UNIT    iCub/Unit.h   ${BML_POSSIBLE_INCDIRS} )


#FIND_PATH(BML_INCLUDE_DIR__CV    _cv.h   ${BML_POSSIBLE_INCDIRS} )


#MESSAGE("DBG BML_INCLUDE_DIR_CV=${BML_INCLUDE_DIR_CV} ")

# find (all) libraries - some dont exist on Linux
FIND_LIBRARY(BML_MAIN_LIBRARY
  NAMES BML 
  PATHS ${BML_POSSIBLE_LIBRARY_PATHS}
  DOC "Location of the main BML "
)

  

########
SET(BML_FOUND ON)
########


###########################################
## CHECK HEADER FILES
# REQUIRED LIBS
FOREACH(INCDIR 
  BML_INCLUDE_DIR_CONNECTION 
  BML_INCLUDE_DIR_ELEMENT	
  BML_INCLUDE_DIR_LAYER
  BML_INCLUDE_DIR_MACHINEBOLTZMANN
  BML_INCLUDE_DIR_ROW
  BML_INCLUDE_DIR_UNIT	
)
  IF    (${INCDIR})
    SET(BML_INCLUDE_DIRS ${BML_INCLUDE_DIRS} ${${INCDIR}} )
	IF(VERBOSE)
	 MESSAGE("${BML_INCLUDE_DIRS}  found for ${INCDIR}.keeping on BML_FOUND ")
	ENDIF (VERBOSE)
  ELSE  (${INCDIR})
	IF(VERBOSE)
	 MESSAGE("${BML_INCLUDE_DIRS} not found turning off BML_FOUND")
	ENDIF(VERBOSE)
    SET(BML_FOUND OFF)
  ENDIF (${INCDIR})  
ENDFOREACH(INCDIR)

SET(BML_INCLUDE_DIRS ${BML_INCLUDE_DIRS} ${${BML_INCLUDE_DIR_CONNECTION}} )
MESSAGE("directory: ${BML_INCLUDE_DIR_CONNECTION}")





# MESSAGE("DBG BML_INCLUDE_DIR=${BML_INCLUDE_DIR}")
# libcxcore does not seem to be always required (different distribution behave 
# differently)
#IF (BML_INCLUDE_DIR_CXCORE)
#  SET(BML_INCLUDE_DIRS ${BML_INCLUDE_DIRS} ${BML_INCLUDE_DIR_CXCORE})
#ELSE (BML_INCLUDE_DIR_CXCORE)
#  IF (WIN32) #required in win
#	SET(BML_FOUND OFF)
	#MESSAGE("- DBG BML_INCLUDE_DIR_CXCORE=${BML_INCLUDE_DIR_CXCORE} ")
#  ENDIF (WIN32)
#ENDIF(BML_INCLUDE_DIR_CXCORE)


#IF(LINK_LIB_HIGHGUI)
#  IF (BML_INCLUDE_DIR_HIGHGUI)
#	SET(BML_INCLUDE_DIRS ${BML_INCLUDE_DIRS} ${BML_INCLUDE_DIR_HIGHGUI})
#  ELSE (BML_INCLUDE_DIR_HIGHGUI)
#	IF (WIN32)
#	  SET(BML_FOUND OFF)
	#MESSAGE("- DBG BML_INCLUDE_DIR_HIGHGUI=${BML_INCLUDE_DIR_HIGHGUI} ")
#	ENDIF (WIN32)
#  ENDIF(BML_INCLUDE_DIR_HIGHGUI)
#ENDIF(LINK_LIB_HIGHGUI)

#################################


## LIBRARIES
# REQUIRED LIBRARIES
FOREACH(LIBNAME  
  BML_MAIN_LIBRARY 
  )
	IF (${LIBNAME})
    		SET(BML_LIBRARIES ${BML_LIBRARIES} ${${LIBNAME}} )
		IF(VERBOSE)
	    		MESSAGE("${LIBNAME} found for ${LIBNAME} .keeping on BML_FOUND")
		ENDIF(VERBOSE)
  	ELSE  (${LIBNAME})
		IF(VERBOSE)
    			MESSAGE("${LIBNAME} not found turning off BML_FOUND")
		ENDIF(VERBOSE)
    SET(BML_FOUND OFF)
  	ENDIF (${LIBNAME})
ENDFOREACH(LIBNAME)

#IF (BML_CXCORE_LIBRARY)
#  SET(BML_LIBRARIES ${BML_LIBRARIES} ${BML_CXCORE_LIBRARY})
#ELSE (BML_CXCORE_LIBRARY)
#  IF (WIN32) #this is required on windows
#	SET(BML_FOUND OFF)
#	MESSAGE("BML_CXCORE_LIBRARY not found turning off BML_FOUND")
#  ENDIF (WIN32)
#ENDIF(BML_CXCORE_LIBRARY)



#IF(LINK_LIB_HIGHGUI)
#  IF (BML_HIGHGUI_LIBRARY)
#	SET(BML_LIBRARIES ${BML_LIBRARIES} ${BML_HIGHGUI_LIBRARY})
#  ELSE (BML_HIGHGUI_LIBRARY)
#	IF (WIN32) #this is required on windows
#	  SET(BML_FOUND OFF)
#	  MESSAGE("BML_HIGHGUI_LIBRARY not found turning off BML_FOUND")
#	ENDIF (WIN32)
#  ENDIF(BML_HIGHGUI_LIBRARY)
#ENDIF(LINK_LIB_HIGHGUI)

# get the link directory for rpath to be used with LINK_DIRECTORIES: 
IF (BML_LIBRARY)
  GET_FILENAME_COMPONENT(BML_LINK_DIRECTORIES ${BML_LIBRARY} PATH)
ENDIF (BML_LIBRARY)


# display help message
IF (NOT BML_FOUND)
	IF(VERBOSE)
  		MESSAGE("BML library or headers not found. "
  		"Please search manually or set env. variable BML_ROOT to guide search." )
	ENDIF(VERBOSE)
ENDIF (NOT BML_FOUND)

MARK_AS_ADVANCED(
  BML_INCLUDE_DIRS  
  BML_LIBRARIES
  BML_LIBRARY
  #BML_HIGHGUI_LIBRARY
  #BML_CVAUX_LIBRARY
  #BML_CXCORE_LIBRARY
  #BML_CVCAM_LIBRARY
  BML_DIR
)

ENDIF (NOT BML_FOUND)

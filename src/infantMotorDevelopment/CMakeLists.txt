PROJECT(dftPrimo)

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

FIND_PACKAGE(OpenCV REQUIRED)
# Ipp location
# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(	${OPENCV_INCLUDE_DIR}
			$ENV{IPP_DIR}/include
			CPlotter
			) 
LINK_DIRECTORIES( 	$ENV{IPP_DIR}/sharedlib
			${OPENCV_LINK_DIRECTORIES}
			)

SUBDIRS( dftCore )


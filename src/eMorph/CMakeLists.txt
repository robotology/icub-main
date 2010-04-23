PROJECT(eMorph)
IF(WIN32)
    CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
    #disable annoying warnings about use of deprecated c functions
    ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
	ADD_DEFINITIONS(/wd4250)
ELSE(WIN32)
    CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
ENDIF(WIN32)

IF(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(NESTED_BUILD TRUE)

SET(EMORPHLIB_DIR "${ICUB_DIR}/src/eMorph/emorph_lib")
ADD_SUBDIRECTORY(emorph_lib)

ADD_SUBDIRECTORY(binFile2yarp)
ADD_SUBDIRECTORY(buffer2image)
ADD_SUBDIRECTORY(cartesian_frame_converter)
ADD_SUBDIRECTORY(cartesian_frame_printer)
ADD_SUBDIRECTORY(logpolar_frame_converter)
ADD_SUBDIRECTORY(logpolar_frame_converter_add)
ADD_SUBDIRECTORY(logpolar_frame_converter_add_gauss)

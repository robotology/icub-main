# If ICUB_FOUND is set, we have already done all this
IF (NOT ICUB_FOUND)

#Changing the default CMAKE_INSTALL_PREFIX
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    "${ICUB_DIR}" CACHE PATH "icub install prefix" FORCE
    )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# Configuration file for the iCub module collection.

# We assume that ICUB_DIR is set.

MESSAGE(STATUS "setting up ICUB")

# pick up iCub's helper modules
SET(ICUB_MODULE_PATH "${ICUB_DIR}/conf")
SET(ICUB_SOURCE_DIR ${ICUB_DIR})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${ICUB_MODULE_PATH})

# some conveniences
INCLUDE(UsePkgConfig)

#Removed, it was causing problems with out of source builds
#of yarp. -Lorenzo Natale 
# try to pick up path to yarp from environment, if set
#IF (NOT YARP_DIR)
#	SET(YARP_DIR "$ENV{YARP_DIR}")
#ENDIF (NOT YARP_DIR)
#IF (NOT YARP_DIR)
#	SET(YARP_ROOT "$ENV{YARP_ROOT}")
#ENDIF (NOT YARP_DIR)

# paths to libraries
SET(PREDICTORS_DIR "${ICUB_DIR}/src/predictors")
SET(CAMERA_DIR "${ICUB_DIR}/src/camera")
SET(SIMIO_DIR "${ICUB_DIR}/src/simio")
SET(ONLINESVR_DIR "${ICUB_DIR}/src/onlineSvr")
SET(YARPMATH_DIR "${ICUB_DIR}/src/yarpMath")
SET(RFWR_DIR "${ICUB_DIR}/src/rfwr")
SET(CVBLOBS_DIR "${ICUB_DIR}/src/cvBlobsLib")
SET(SPMAP_DIR "${ICUB_DIR}/src/spMap")
SET(KINEMATICS_DIR "${ICUB_DIR}/src/kinematics")
SET(FASTFILT_DIR "${ICUB_DIR}/src/fastFilt")
SET(SALIENCELIB_DIR "${ICUB_DIR}/src/salience")
SET(EGOSPHERELIB_DIR "${ICUB_DIR}/src/egoSphere")
SET(ATTENTIONSELECTIONLIB_DIR "${ICUB_DIR}/src/attentionSelection")
SET(DIGITALPID_DIR "${ICUB_DIR}/src/digitalPid")
SET(SHARKSWITHLASERS_DIR "${ICUB_DIR}/src/sharksWithLasers")
SET(LOGPOLAR_DIR "${ICUB_DIR}/src/logPolar")
SET(FOURIERVISION_DIR "${ICUB_DIR}/src/fourierVision")
SET(OPTFLOW_DIR "${ICUB_DIR}/src/optFlow")
SET(CANLOADERLIB_DIR "${ICUB_DIR}/src/canLoader/canLoader2.0/canLoaderLib")
SET(YARPQWIDGETS_DIR "${ICUB_DIR}/src/gui/libYARP_QWidgets")
SET(ICUBQWIDGETS_DIR "${ICUB_DIR}/src/gui/libICUB_QWidgets")
SET(PPEVENTDEBUGGER_DIR "${ICUB_DIR}/src/ppEventDebugger")
SET(RDOUT_DIR "${ICUB_DIR}/src/rdout")
SET(IKIN_DIR "${ICUB_DIR}/src/iKin")
SET(BML_DIR "${ICUB_DIR}/src/boltzmannMachineLibrary/bml")
SET(CTRLLIB_DIR "${ICUB_DIR}/src/ctrlLib")

SET(icubmod_DIR "${ICUB_DIR}")
SET(ICUBDEV_DIR "${ICUB_DIR}/src/iCubDev")

SET(ICUB_FOUND TRUE)
ENDIF (NOT ICUB_FOUND)

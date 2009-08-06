PROJECT (iCub)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

SET(ROBOT_NAME "$ENV{ICUB_ROBOTNAME}" CACHE PATH
	"Name of your robot")

SET(ROBOT_DIR "$ENV{ICUB_ROOT}/app/${ROBOT_NAME}")
SET(ICUB_APP "$ENV{ICUB_ROOT}/app")

IF ("${ROBOT_NAME}" STREQUAL "")
  MESSAGE(STATUS "No robot name")
ELSE ("${ROBOT_NAME}" STREQUAL "")
  IF (EXISTS ${ROBOT_DIR})
	MESSAGE(STATUS "Setting up install for ${ROBOT_NAME}")
	ADD_SUBDIRECTORY(${ROBOT_NAME})
  ENDIF(EXISTS ${ROBOT_DIR})
ENDIF ("${ROBOT_NAME}" STREQUAL "")

ADD_SUBDIRECTORY(default)
#ADD_SUBDIRECTORY(attentionDistributed)
#ADD_SUBDIRECTORY(iha)
#ADD_SUBDIRECTORY(demoy3)
#ADD_SUBDIRECTORY(crossPowerSpectrumVergence)
#ADD_SUBDIRECTORY(armCartesianController)
#ADD_SUBDIRECTORY(faceExpressions)








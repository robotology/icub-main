PROJECT (iCub)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

SET(ROBOT_NAME "$ENV{ICUB_ROBOTNAME}" CACHE PATH
	"Name of your robot")

SET(ROBOT_DIR "$ENV{ICUB_ROOT}/app/${ROBOT_NAME}")
SET(ICUB_APP "$ENV{ICUB_DIR}/app")

IF ("${ROBOT_NAME}" STREQUAL "")
  MESSAGE(STATUS "No robot name")
ELSE ("${ROBOT_NAME}" STREQUAL "")
  IF (EXISTS ${ROBOT_DIR})
	# MESSAGE(STATUS "Setting up install for ${ROBOT_NAME}")
	# ADD_SUBDIRECTORY(${ROBOT_NAME})
  ENDIF(EXISTS ${ROBOT_DIR})
ENDIF ("${ROBOT_NAME}" STREQUAL "")

add_custom_command(OUTPUT copy
					COMMAND  ${CMAKE_COMMAND} -E echo "Dummy command1"
					COMMENT "Running copy command"
					)
					
include("iCubAddApplication.cmake")
iCubAddApplication(default ${ICUB_APP})

add_custom_target(INSTALL_APPLICATIONS DEPENDS copy)

#ADD_SUBDIRECTORY(attentionDistributed)
#ADD_SUBDIRECTORY(iha)
#ADD_SUBDIRECTORY(demoy3)
#ADD_SUBDIRECTORY(crossPowerSpectrumVergence)
#ADD_SUBDIRECTORY(armCartesianController)
#ADD_SUBDIRECTORY(faceExpressions)








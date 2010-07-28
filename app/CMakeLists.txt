PROJECT (iCub)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

SET(ROBOT_NAME "$ENV{ICUB_ROBOTNAME}" CACHE PATH
	"Name of your robot")

SET(ROBOT_DIR "$ENV{ICUB_ROOT}/app/${ROBOT_NAME}")


set(dummy ${CMAKE_BINARY_DIR}/f-app-main)
add_custom_command(OUTPUT ${dummy}
					COMMAND ${CMAKE_COMMAND} -E make_directory ${ICUB_DESTINATION_APP}
					COMMAND ${CMAKE_COMMAND} -E make_directory ${ICUB_DESTINATION_APP}/app
					COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ICUB_ROOT.ini ${ICUB_DESTINATION_APP}
					COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/icubapp.py ${ICUB_DESTINATION_APP}/app/
					COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/app.txt.template ${ICUB_DESTINATION_APP}/app/
					COMMENT "Setting up application directory"
					)
					
add_subdirectory(default)

IF ("${ROBOT_NAME}" STREQUAL "")
  MESSAGE(STATUS "No robot name")
ELSE ("${ROBOT_NAME}" STREQUAL "")
  IF (EXISTS ${ROBOT_DIR})
	# MESSAGE(STATUS "Setting up install for ${ROBOT_NAME}")
	add_subdirectory(${ROBOT_NAME})
  ENDIF(EXISTS ${ROBOT_DIR})
ENDIF ("${ROBOT_NAME}" STREQUAL "")


add_custom_target(app ALL DEPENDS ${dummy})

icub_set_property(GLOBAL APPEND PROPERTY ICUB_APPLICATIONS app)
icub_set_property(GLOBAL APPEND PROPERTY ICUB_APPLICATIONS_FILES ${dummy})


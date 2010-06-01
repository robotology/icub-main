add_subdirectory(simCartesianControl)

if(ICUB_USE_ODE AND ICUB_USE_SDL)
	add_subdirectory(iCubSimulation)
else(ICUB_USE_ODE AND ICUB_USE_SDL)
	message(STATUS "skipping iCubSimulation")
endif(ICUB_USE_ODE AND ICUB_USE_SDL)



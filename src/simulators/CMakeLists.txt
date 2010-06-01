add_subdirectory(simCartesianControl)

if(ICUB_USE_ODE AND ICUB_USE_SDL AND ICUB_USE_OpenGL)
	add_subdirectory(iCubSimulation)
else(ICUB_USE_ODE AND ICUB_USE_SDL AND ICUB_USE_OpenGL)
	message(status "skipping iCubSimulation")
endif(ICUB_USE_ODE AND ICUB_USE_SDL AND ICUB_USE_OpenGL)



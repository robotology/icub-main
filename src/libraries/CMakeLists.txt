add_subdirectory(icubmod)
add_subdirectory(iCubDev)
add_subdirectory(logPolar)
	
if (ICUB_USE_GSL)
	add_subdirectory(ctrlLib)
	add_subdirectory(iKin)
    add_subdirectory(affActionPrimitives)
endif (ICUB_USE_GSL)



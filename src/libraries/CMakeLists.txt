add_subdirectory(icubmod)
add_subdirectory(iCubDev)
	
if (ICUB_USE_GSL)
	add_subdirectory(ctrlLib)
	add_subdirectory(iKin)
endif (ICUB_USE_GSL)



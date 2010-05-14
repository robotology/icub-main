add_subdirectory(icubmod)

if (ICUB_USE_GSL)
	add_subdirectory(ctrlLib)
	add_subdirectory(iKin)
endif (ICUB_USE_GSL)



add_subdirectory(icubmod)

if (ICUB_HAS_GSL)
	add_subdirectory(ctrlLib)
endif (ICUB_HAS_GSL)

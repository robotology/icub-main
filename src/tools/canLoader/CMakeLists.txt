PROJECT(canLoader)

if(ICUB_USE_GtkPlus)
	ADD_SUBDIRECTORY(canLoader)
else(ICUB_USE_GtkPlus)
	message(STATUS "GtkPlus not selected, skipping canLoader")
endif(ICUB_USE_GtkPlus)

ADD_SUBDIRECTORY(canLoaderLib)


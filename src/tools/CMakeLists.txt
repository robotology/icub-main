add_subdirectory(controlBoardDumper)
add_subdirectory(dataDumper)
add_subdirectory(testStereoMatch)
add_subdirectory(velocityObserver)

if (ICUB_USE_GtkPlus)
	add_subdirectory(canLoader)
else (ICUB_USE_GtkPlus)
	message(status "GtkPlus not selected, skipping canLoader")
endif(ICUB_USE_GtkPlus)


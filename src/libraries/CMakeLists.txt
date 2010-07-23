#order matters, put libs in dependency order

if(ICUB_USE_OpenCV)
    add_subdirectory(iCubVis)
else(ICUB_USE_OpenCV)
    message(STATUS "OpenCV not found/selected, skipping iCubVis")
endif(ICUB_USE_OpenCV)

add_subdirectory(iCubDev)

add_subdirectory(icubmod)

add_subdirectory(ctrlLib)

add_subdirectory(iKin)

add_subdirectory(iDyn)

add_subdirectory(logPolar)

add_subdirectory(actionPrimitives)

add_subdirectory(learningMachine)

add_subdirectory(spMap)



##################
# IMPORTANT: the order with which you add directories 
# here matters. 
# Put libraries in dependency order. If A depends from B
# put B before A. Failure to do so might break the build.
# 

if(ICUB_USE_OpenCV)
    add_subdirectory(iCubVis)
else(ICUB_USE_OpenCV)
    message(STATUS "OpenCV not found/selected, skipping iCubVis")
endif(ICUB_USE_OpenCV)

add_subdirectory(iCubDev)
add_subdirectory(logpolar)
add_subdirectory(ctrlLib)
add_subdirectory(iKin)
add_subdirectory(iDyn)
add_subdirectory(actionPrimitives)
add_subdirectory(learningMachine)
add_subdirectory(icubmod)



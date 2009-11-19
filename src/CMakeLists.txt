PROJECT (iCub)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.

FIND_PACKAGE(YARP REQUIRED)

FIND_PACKAGE(OpenCV QUIET)
IF (NOT OPENCV_FOUND)
   MESSAGE(STATUS "Skipping modules using OpenCV (Open Computer Vision)")
ENDIF (NOT OPENCV_FOUND)

FIND_PACKAGE(IPOPT QUIET)
IF (NOT IPOPT_FOUND)
   MESSAGE(STATUS "Skipping modules using IPOPT")
ENDIF (NOT IPOPT_FOUND)

FIND_PACKAGE(GSL QUIET)
IF (NOT GSL_FOUND)
   MESSAGE(STATUS "Skipping modules using GSL (GNU Scientific Library)")
ENDIF (NOT GSL_FOUND)

FIND_PACKAGE(ODE QUIET)
IF (NOT ODE_FOUND)
   MESSAGE(STATUS "Skipping modules using ODE (Open Dynamics Engine)")
ENDIF (NOT ODE_FOUND)

#find_package(LIBVISLAB QUIET)
#IF (NOT LIBVISLAB_FOUND)
#   MESSAGE(STATUS "Skipping modules using OpenVislab (http://OpenVislab.sf.net) libraries")
#ENDIF (NOT LIBVISLAB_FOUND)

FIND_PACKAGE(SDL QUIET)

# SDL_FOUND variable seems unreliable!
IF (SDL_LIBRARY)
  SET(SDL_FOUND "YES")
ENDIF (SDL_LIBRARY)
IF (NOT SDL_FOUND)
  MESSAGE(STATUS "Skipping modules using SDL (Simple DirectMedia Library)")
ENDIF (NOT SDL_FOUND)

IF (CREATE_GUIS_GTK)
	FIND_PACKAGE(GtkPlus QUIET)
		IF (NOT GtkPlus_FOUND)
  			MESSAGE("You have requested to build guis that require GtkPlus, but GtkPlus was not found. Install GtkPlus or turn CREATE_GUIS_GTK off")
		ENDIF (NOT GtkPlus_FOUND)
ENDIF(CREATE_GUIS_GTK)

IF(CREATE_GUIS_GTKMM)
	FIND_PACKAGE(GtkMM QUIET)
		IF (NOT GtkMM_FOUND)
		  MESSAGE("You have requested to build guis that require Gtkmm, but Gtkmm was not found. Install GtkMM or turn CREATE_GUIS_GTKMM off")
		ENDIF (NOT GtkMM_FOUND)
ENDIF(CREATE_GUIS_GTKMM)

# Formal releases of RobotCub software are starting.
# In releases, the "BUILD_FOR_RELEASE" flag is set.
SET(BUILD_FOR_RELEASE FALSE CACHE BOOL "Is this an official build?")
MARK_AS_ADVANCED(BUILD_FOR_RELEASE)  # not for general use
IF (BUILD_FOR_RELEASE)
  # Only a few demo projects are included so far.
  ADD_SUBDIRECTORY(iCubSimulation)
  ADD_SUBDIRECTORY(iCubDemoY3)
  ADD_SUBDIRECTORY(gui/robotMotorGui)
ELSE (BUILD_FOR_RELEASE)

# icub modules, added by default.
IF (USE_ICUB_MOD)
  ADD_SUBDIRECTORY(modules)
  ADD_SUBDIRECTORY(iCubDev)
  IF (NOT SKIP_logpolarclient)
    ADD_SUBDIRECTORY(logpolarRemapper)
  ENDIF (NOT SKIP_logpolarclient)
ENDIF (USE_ICUB_MOD)

ADD_SUBDIRECTORY(rdout)
ADD_SUBDIRECTORY(rdin)

FIND_PACKAGE(ARToolkit QUIET)
IF (ARTOOLKIT_FOUND)  
  ADD_SUBDIRECTORY(artoolkittracker)
  ADD_SUBDIRECTORY(demoafford)
ELSEIF (ARTOOLKIT_FOUND)
  MESSAGE(STATUS "Skipping modules using ARToolkit")
ENDIF (ARTOOLKIT_FOUND)

FIND_PACKAGE(ARToolkitPlus QUIET)
IF (ARTOOLKITPLUS_FOUND)
  ADD_SUBDIRECTORY(artkpTrackSingleMarker)
ELSEIF (ARTOOLKITPLUS_FOUND)
  MESSAGE(STATUS "Skipping modules using ARToolkitPlus")
ENDIF (ARTOOLKITPLUS_FOUND)
    
  ADD_SUBDIRECTORY(headMover)
  ADD_SUBDIRECTORY(headMoverManual)
  ADD_SUBDIRECTORY(emotionInterface) 
  ADD_SUBDIRECTORY(iCubInterface2)
  ADD_SUBDIRECTORY(simpleClient)
  ADD_SUBDIRECTORY(learner)
  ADD_SUBDIRECTORY(simpleHeadControl)
  #ADD_SUBDIRECTORY(stdModel)
  ADD_SUBDIRECTORY(tracker)
  ADD_SUBDIRECTORY(canbusCollector)
  ADD_SUBDIRECTORY(dataDumper)
  ADD_SUBDIRECTORY(controlBoardDumper)
  ADD_SUBDIRECTORY(velocityControl)  
  ADD_SUBDIRECTORY(testStereoMatch)
  ADD_SUBDIRECTORY(iCubDemoY3)  
    
  IF (OPENCV_FOUND)
    ADD_SUBDIRECTORY(salience)
    ADD_SUBDIRECTORY(visualBlobs)
    ADD_SUBDIRECTORY(camCalib)
    ADD_SUBDIRECTORY(camCalibConf)
    ADD_SUBDIRECTORY(camExtrinsicsCalib)    
    IF (GSL_FOUND)
      ADD_SUBDIRECTORY(attentionSelection)      
      #ADD_SUBDIRECTORY(controlGaze)
      ADD_SUBDIRECTORY(controlGaze2)
      ADD_SUBDIRECTORY(egoSphere)
      ADD_SUBDIRECTORY(attentionLogger)
      ADD_SUBDIRECTORY(demoAffv2)   
    ENDIF (GSL_FOUND)
    
    #   ADD_SUBDIRECTORY(histTracker)
    
    ADD_SUBDIRECTORY(faceEyeDetector)
    ADD_SUBDIRECTORY(soundLocalization)
    
    ADD_SUBDIRECTORY(camShiftPlus)
    
    ADD_SUBDIRECTORY(cvBlobsLib)
    ADD_SUBDIRECTORY(fastFilt)
    ADD_SUBDIRECTORY(faceTracker)
    
  ENDIF (OPENCV_FOUND)

  # A basic example of a module, a library, and how to use the library
  # from the module.
  ADD_SUBDIRECTORY(sharksWithLasers)
  ADD_SUBDIRECTORY(takeOverTheWorld)
  
  # can loader stuff, internally check available libraries
  ADD_SUBDIRECTORY(canLoader)
  # assorted GUIs, internally check available libraries
  ADD_SUBDIRECTORY(gui)
  
  # some less tested modules, only compiled if ICUB_TEST is set in environment
  SET (ICUB_TEST "$ENV{ICUB_TEST}")
  IF (ICUB_TEST)
    ADD_SUBDIRECTORY(ballTracker)
    ADD_SUBDIRECTORY(colorSegmentation)
  ENDIF (ICUB_TEST)
  
  # libraries -- adding them last to test if CMake reorders correctly
  ADD_SUBDIRECTORY(rfwr)
  ADD_SUBDIRECTORY(yarpMath)
  ADD_SUBDIRECTORY(simio)
  ADD_SUBDIRECTORY(spMap)
  ADD_SUBDIRECTORY(predictors)  

  IF (GSL_FOUND)
    ADD_SUBDIRECTORY(kinematics) 
    ADD_SUBDIRECTORY(ctrlLib)
    ADD_SUBDIRECTORY(iKin)
    ADD_SUBDIRECTORY(iKinHead)
    ADD_SUBDIRECTORY(velocityObserver)  
    ADD_SUBDIRECTORY(scriptSequencer)
    ADD_SUBDIRECTORY(demoReachManager_IIT_ISR)
  ENDIF (GSL_FOUND)
  
  IF (ODE_FOUND)
    IF (SDL_FOUND)
      ADD_SUBDIRECTORY(iCubSimulation)
    ENDIF (SDL_FOUND)
  ENDIF (ODE_FOUND)
  
  # vergence modules
  ADD_SUBDIRECTORY(crossPowerSpectrumVergence)
  ADD_SUBDIRECTORY(fourierVision)
  ADD_SUBDIRECTORY(logPolar)
  ADD_SUBDIRECTORY(rectification)

  # cognitive architecture modules
  ADD_SUBDIRECTORY(logPolarTransform)
  ADD_SUBDIRECTORY(demoAAM)
  IF (OPENCV_FOUND)
     ADD_SUBDIRECTORY(autoAssociativeMemory)
     ADD_SUBDIRECTORY(episodicMemory)
  ENDIF (OPENCV_FOUND)

  # utility modules
  ADD_SUBDIRECTORY(imageSource)

  # example module that is compliant with iCub configuration, coding, and documentation standards
  ADD_SUBDIRECTORY(myModule)

  # optical flow modules
  #ADD_SUBDIRECTORY(optFlow) //does not compile on LINUX yet


  IF(GSL_FOUND AND IPOPT_FOUND)    
    ADD_SUBDIRECTORY(iKinCartesianSolver)
    ADD_SUBDIRECTORY(iKinArmCtrl)
    ADD_SUBDIRECTORY(iKinGazeCtrl)    
    ADD_SUBDIRECTORY(cylGuard)
  ENDIF(GSL_FOUND AND IPOPT_FOUND)

  ADD_SUBDIRECTORY(iKinArmCtrlIF)
  ADD_SUBDIRECTORY(affActionPrimitives)

  if(LIBVISLAB_FOUND)
    add_subdirectory(abstractHandCtrl)

    find_package(IKIN QUIET)
    if(IKIN_FOUND)
      add_subdirectory(eye2world)
    endif(IKIN_FOUND)
  endif(LIBVISLAB_FOUND)

  # add here modules you want to test for compilation
  # these modules will only be compiled by automated
  # compilation scripts
  IF (COMPILE_FOR_TESTING)

  ENDIF(COMPILE_FOR_TESTING)

IF(OPENCV_FOUND AND IPOPT_FOUND)
    ADD_SUBDIRECTORY(pf3dTracker)
ENDIF(OPENCV_FOUND AND IPOPT_FOUND)
  
ENDIF(BUILD_FOR_RELEASE)


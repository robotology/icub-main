

IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(skinprototype TYPE SkinPrototype INCLUDE skinprototype.h
				WRAPPER grabber)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_static_grabber)
FIND_PACKAGE(YARP REQUIRED)

ADD_LIBRARY(skinprototype skinprototype.h skinprototype.cpp)
icub_export_library(skinprototype "" include/iCub "")
ENDIF (NOT SKIP_static_grabber)

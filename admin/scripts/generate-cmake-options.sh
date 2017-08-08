### parameters are 
# $1: hostname (could be also buildtype)
# $2: os type (macos, winxp, lenny, etch, karmic ...)
# $3: test type: nightly continuous experimental


CMAKE_OPTIONS="\
   -DTEST_MACHINE_HOSTNAME:STRING=$1 \
   -DTEST_MACHINE_OS_TYPE:STRING=$2 \
   -DTEST_MACHINE_TEST_TYPE:STRING=$3 \
   -DBUILD_TESTING:BOOL=TRUE \
   -DENABLE_icubmod_cartesiancontrollerclient:BOOL=TRUE \
   -DENABLE_icubmod_cartesiancontrollerserver:BOOL=TRUE \
   -DENABLE_icubmod_gazecontrollerclient:BOOL=TRUE \
   -DENABLE_icubmod_icubarmcalibrator:BOOL=TRUE \
   -DENABLE_icubmod_icubarmcalibratorj8:BOOL=TRUE \
   -DENABLE_icubmod_icubhandcalibrator:BOOL=TRUE \
   -DENABLE_icubmod_icublegscalibrator:BOOL=TRUE \
   -DENABLE_icubmod_logpolarclient:BOOL=TRUE \
   -DENABLE_icubmod_logpolargrabber:BOOL=TRUE \
   -DENABLE_icubmod_skinprototype:BOOL=TRUE \
   -DENABLE_icubmod_fakecan:BOOL=TRUE \
   -DENABLE_icubmod_canmotioncontrol:BOOL=FALSE\
   -DENABLE_icubmod_debugInterfaceClient:BOOL=TRUE\
   -DENABLE_icubmod_usbCamera:BOOL=TRUE \
   -DENABLE_icubmod_usbCameraRaw:BOOL=TRUE \
   -DICUB_DASHBOARD_SUBMIT:BOOL=TRUE"

case $3 in
   "Experimental" )
      CMAKE_OPTIONS=" \
        -DICUB_ICUBINTERFACE_EXPERIMENTAL:BOOL=TRUE \
        $CMAKE_OPTIONS \
      " 
     ;;
   "Continuous" )
      CMAKE_OPTIONS=" \
        $CMAKE_OPTIONS \
      " 
     ;;
   "Nightly" )
      CMAKE_OPTIONS=" \
        $CMAKE_OPTIONS \
      " 
     ;;
esac

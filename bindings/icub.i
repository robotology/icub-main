// Copyright: (C) 2011 IITRBCS
// Authors: Paul Fitzpatrick
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

%module icub

%include "std_string.i"
%include "std_vector.i"

%import "yarp.i"

%{
#include <yarp/dev/Drivers.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
using namespace yarp::os;
using namespace yarp::sig;

// iKin
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinHlp.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinSlv.h>

// ctrlLib
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/clustering.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/functionEncoder.h>
#include <iCub/ctrl/kalman.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/neuralNetworks.h>
#include <iCub/ctrl/optimalControl.h>
#include <iCub/ctrl/outliersDetection.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/tuning.h>

// skinDynLib
#include <iCub/skinDynLib/common.h>
#include <iCub/skinDynLib/Taxel.h>
#include <iCub/skinDynLib/skinPart.h>
#include <iCub/skinDynLib/iCubSkin.h>
#include <iCub/skinDynLib/dynContact.h>
#include <iCub/skinDynLib/dynContactList.h>
#include <iCub/skinDynLib/rpcSkinManager.h>
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>


// iDyn
//#include <iCub/iDyn/iDynInv.h>
//#include <iCub/iDyn/iDyn.h>
//#include <iCub/iDyn/iDynContact.h>
//#include <iCub/iDyn/iDynTransform.h>
//#include <iCub/iDyn/iDynBody.h>

%}

%include <std_vector.i>

// iKin
%include <iCub/iKin/iKinFwd.h>
%include <iCub/iKin/iKinHlp.h>
%include <iCub/iKin/iKinInv.h>
%include <iCub/iKin/iKinIpOpt.h>
%include <iCub/iKin/iKinSlv.h>

// ctrlLib
%include <iCub/ctrl/adaptWinPolyEstimator.h>
%include <iCub/ctrl/clustering.h>
%include <iCub/ctrl/filters.h>
%include <iCub/ctrl/functionEncoder.h>
%include <iCub/ctrl/kalman.h>
%include <iCub/ctrl/math.h>
%include <iCub/ctrl/minJerkCtrl.h>
%include <iCub/ctrl/neuralNetworks.h>
%include <iCub/ctrl/optimalControl.h>
%include <iCub/ctrl/outliersDetection.h>
%include <iCub/ctrl/pids.h>
%include <iCub/ctrl/tuning.h>

// skinDynLib
%include <iCub/skinDynLib/common.h>
%include <iCub/skinDynLib/Taxel.h>
%include <iCub/skinDynLib/skinPart.h>
%include <iCub/skinDynLib/iCubSkin.h>
%include <iCub/skinDynLib/dynContact.h>
%include <iCub/skinDynLib/dynContactList.h>
%include <iCub/skinDynLib/rpcSkinManager.h>
%include <iCub/skinDynLib/skinContact.h>
%include <iCub/skinDynLib/skinContactList.h>


// iDyn
//%include <iCub/iDyn/iDynInv.h>

//%ignore notImplemented;
//%include <iCub/iDyn/iDyn.h>
//%include <iCub/iDyn/iDynContact.h>
//%include <iCub/iDyn/iDynTransform.h>
//%include <iCub/iDyn/iDynBody.h>

%{
typedef yarp::os::TypedReader<iCub::skinDynLib::skinContactList> TypedReaderSkinContactList;
typedef yarp::os::TypedReaderCallback<iCub::skinDynLib::skinContactList> TypedReaderCallbackSkinContactList;
typedef yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> BufferedPortSkinContactList;
%}

%{
typedef yarp::os::TypedReader<iCub::skinDynLib::dynContactList> TypedReaderDynContactList;
typedef yarp::os::TypedReaderCallback<iCub::skinDynLib::dynContactList> TypedReaderCallbackDynContactList;
typedef yarp::os::BufferedPort<iCub::skinDynLib::dynContactList> BufferedPortDynContactList;
%}

%inline %{
    typedef size_t size_type;
%}

%template(TypedReaderSkinContactList) yarp::os::TypedReader<iCub::skinDynLib::skinContactList>;
%template(TypedReaderCallbackSkinContactList) yarp::os::TypedReaderCallback<iCub::skinDynLib::skinContactList>;
%template(BufferedPortSkinContactList) yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>;

%template(TypedReaderDynContactList) yarp::os::TypedReader<iCub::skinDynLib::dynContactList>;
%template(TypedReaderCallbackDynContactList) yarp::os::TypedReaderCallback<iCub::skinDynLib::dynContactList>;
%template(BufferedPortDynContactList) yarp::os::BufferedPort<iCub::skinDynLib::dynContactList>;


bool init();

%{
  bool init() {
      return true;
  }
%}


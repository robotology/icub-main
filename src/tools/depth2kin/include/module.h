/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __DEPTH2KIN_MODULE_H__
#define __DEPTH2KIN_MODULE_H__

#include <string>
#include <deque>

#include <cv.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <iCub/iKin/iKinFwd.h>

#include "depth2kin_IDL.h"
#include "methods.h"
#include "nlp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::iKin;

// forward declaration
class CalibModule;


/************************************************************************/
class DisparityProcessor : public BufferedPort<ImageOf<PixelMono> >
{
protected:
    CalibModule *module;
    void onRead(ImageOf<PixelMono> &imgIn);

public:
    DisparityProcessor(CalibModule *module);
};



/************************************************************************/
class CalibModule : public RFModule, public depth2kin_IDL
{
protected:
    ResourceFinder    *rf;
    PolyDriver         drvArmL;
    PolyDriver         drvArmR;
    PolyDriver         drvCartL;
    PolyDriver         drvCartR;
    PolyDriver         drvGaze;
    IEncoders         *iencs;
    IPositionControl  *iposs;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
        
    Mutex mutex;
    iCubFinger finger;
    Vector curExplorationCenter;

    Calibrator *calibrator;
    LocallyWeightedExperts expertsL,expertsR,*experts;
    EyeAligner aligner;

    string arm;
    bool   useArmL,useArmR;
    bool   selectArmEnabled;
    double max_dist;
    double block_eyes;
    double exploration_wait;
    int    roi_side;
    int    nEncs;
    int    test;
    bool   enabled;
    bool   calibrated;
    bool   isSaved;
    bool   closing;    
    bool   exp_depth2kin;
    bool   exp_aligneyes;
    bool   touchWithExperts;

    deque<Vector> targets,targetsConsumed;
    bool   motorExplorationAsyncStop;
    enum { motorExplorationStateIdle,
           motorExplorationStateTrigger,
           motorExplorationStateReach,
           motorExplorationStateLog } motorExplorationState;

    int context_arm;
    int context_gaze;

    BufferedPort<Bottle>             touchInPort;
    DisparityProcessor               depthInPort;
    BufferedPort<ImageOf<PixelBgr> > depthOutPort;
    RpcClient                        depthRpcPort;
    RpcServer                        rpcPort;

    bool attach(RpcServer &source);
    bool createTargets(const Vector &c, const Vector &size);
    bool isTypeValid(const string &type);
    Calibrator *factory(const string &type);
    bool factory(Value &v);
    cv::Rect extractFingerTip(ImageOf<PixelMono> &imgIn, ImageOf<PixelBgr> &imgOut,
                              const Vector &c, Vector &px);
    double getMinVer() const;
    bool getGazeParams(const string &eye, const string &type, Matrix &M);
    bool pushExtrinsics(const string &eye, const Matrix &H);
    bool getDepth(const Vector &px, Vector &x, Vector &pxr);
    bool getDepthAveraged(const Vector &px, Vector &x, Vector &pxr, const int maxSamples=5);
    void openHand(IPositionControl *ipos);
    void postureHelper(const Vector &gaze_ang, const Matrix &targetL, const Matrix &targetR);
    void prepareRobot();
    int removeOutliers();
    void doMotorExploration();
    void doTouch(const Vector &xd);
    void doTest();

public:
    CalibModule();
    bool configure(ResourceFinder &rf);
    void onRead(ImageOf<PixelMono> &imgIn);
    double getPeriod();
    bool updateModule();
    void terminate();
    bool close();

    // IDL methods
    int getNumExperts();
    bool clearExperts();
    bool load();
    bool save();
    bool log(const string &type);
    bool explore();
    bool stop();
    bool setMaxDist(const double max_dist);
    double getMaxDist();
    bool setRoi(const int side);
    int getRoi();
    bool setBlockEyes(const double block_eyes);
    double getBlockEyes();
    bool blockEyes();
    bool clearEyes();
    bool setArm(const string &arm);
    string getArm();
    bool setCalibrationType(const string &type, const string &extrapolation);
    string getCalibrationType();
    Property calibrate(const bool rm_outliers);
    bool pushCalibrator();
    bool setTouchWithExperts(const string &sw);
    string getTouchWithExperts();
    bool touch(const int u, const int v);
    PointReq getPoint(const string &arm, const double x, const double y, const double z);
    bool setExperiment(const string &exp, const string &v);
    string getExperiment(const string &exp);
    Vector getExtrinsics(const string &eye);
    bool resetExtrinsics(const string &eye);
    bool setExplorationWait(const double wait);
    double getExplorationWait();
    bool setExplorationSpace(const double cx, const double cy, const double cz,
                             const double a, const double b);
    bool setExplorationSpaceDelta(const double dcx, const double dcy, const double dcz,
                                  const double da, const double db);
    Property getExplorationData();
    bool clearExplorationData();
    bool posture(const string &type);
    bool calibrateDepth();
    bool quit();
};


#endif



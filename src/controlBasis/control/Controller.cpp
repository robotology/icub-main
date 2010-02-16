// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "Controller.h"
#include <yarp/os/Network.h>
#include <yarp/math/Math.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

CB::Controller::Controller(ControlBasisResource *sen,
                           ControlBasisResource *ref,
                           ControlBasisPotentialFunction *pf,
                           ControlBasisResource *eff) : 
    ControlBasisAction(),
    distributedMode(false),
    sensor(sen),
    reference(ref),
    effector(eff),
    potentialFunction(pf),
    potentialDot(0),
    potentialLast(0),
    needsJacobian(false),
    needsJacobianInverse(false),
    hasReference(true),
    gain(1.0),
    useJacobianTranspose(true),
    jacobian(NULL)
{
    
    string refType = "";

    
    // get the types of the resources
    inputSpace = sensor->getResourceType();
    outputSpace = effector->getResourceType();
    pfType = potentialFunction->getSpace();
    
    if(ref != NULL) {
        cout << "Controller HAS reference" << endl;
        hasReference = true;
        reference = ref;
        refType = reference->getResourceType();
        if(refType != inputSpace) {
            cout << "Controller reference and sensor signal don't match (" << refType.c_str() << "," << inputSpace.c_str() << ")!!\n\n";
            exit(0);
        }
    }
    
    if(pfType!=outputSpace) {
        needsJacobian = true;
        cout << "Controller doesn't know how to handle jacobians yet!! -- exiting!!" << endl;
        exit(0);
    } 
    
    actionName = "/cb/controller/";
    actionName += "s:" + sensor->getResourceName() + "/";
    if(hasReference) {
        actionName += "r:" + reference->getResourceName() + "/";
    }
    actionName += "e:" + effector->getResourceName() + "/";
    actionName += "pf:" + potentialFunction->getPotentialName();
    
    numOutputs = 1;
    outputNames.push_back("u");
    
    int inputSize = sensor->getResourceDataSize();
    int outputSize = effector->getResourceDataSize();
    Vout.resize(outputSize);
    
}

CB::Controller::Controller(ControlBasisResource *sen,
                           ControlBasisPotentialFunction *pf,
                           ControlBasisResource *eff) :
    ControlBasisAction(),
    distributedMode(false),
    sensor(sen),
    reference(NULL),
    effector(eff),
    potentialFunction(pf),
    potentialDot(0),
    potentialLast(0),
    needsJacobian(false),
    needsJacobianInverse(false),
    hasReference(false),
    gain(1.0),
    useJacobianTranspose(true),
    jacobian(NULL)
{    

    string refType = "";

    // get the types of the resources
    inputSpace = sensor->getResourceType();
    outputSpace = effector->getResourceType();
    pfType = potentialFunction->getSpace();
    
    if(pfType!=outputSpace) {
        needsJacobian = true;
        cout << "Controller doesn't know how to handle jacobians yet!! -- exiting!!" << endl;
        exit(0);
    } 
    
    actionName = "/cb/controller/";
    actionName += "s:" + sensor->getResourceName() + "/";
    actionName += "e:" + effector->getResourceName() + "/";
    actionName += "pf:" + potentialFunction->getPotentialName();
    
    numOutputs = 1;
    outputNames.push_back("u");
    
    int inputSize = sensor->getResourceDataSize();
    int outputSize = effector->getResourceDataSize();
    Vout.resize(outputSize);
    
}

CB::Controller::Controller(string sen, string ref, string pf, string eff) :
    sensorName(sen),
    referenceName(ref),
    effectorName(eff),
    pfName(pf),
    distributedMode(true),
    needsJacobian(false),
    needsJacobianInverse(false),
    hasReference(true),
    gain(1.0),
    jacobian(NULL),
    potentialFunction(NULL),
    useJacobianTranspose(true),
    sensor(NULL),
    reference(NULL),
    effector(NULL)
{

    // do some string ops to get valid names 
    parseOutputResource();    

    if(!createPotentialFunction(pf)) {
        cout << "Controller can't create potential function!!" << endl;
        return;
    }    

    cout << "checking if controller needs jacobian..." << endl;
    if(outputSpace != inputSpace) {
        needsJacobian = true;
        if(!createJacobian()) {
            cout << "Controller can't create Jacobian!!" << endl;
            return;
        }    
    }
    cout << "Controller needs jacobian: " << needsJacobian << endl;

    actionName = "/cb/controller/";
    actionName += "s:" + sensorName + "/";
    actionName += "r:" + referenceName + "/";
    actionName += "e:" + effectorName + "/";
    actionName += "pf:" + pfName;


    
}

CB::Controller::Controller(string sen, string pf, string eff) :
    sensorName(sen),
    referenceName(""),
    effectorName(eff),
    pfName(pf),
    distributedMode(true),
    needsJacobian(false),
    needsJacobianInverse(false),
    hasReference(false),
    gain(1.0),
    jacobian(NULL),
    potentialFunction(NULL),
    useJacobianTranspose(true),
    sensor(NULL),
    reference(NULL),
    effector(NULL)
{

    // do some string ops to get valid names 
    parseOutputResource();    

    if(!createPotentialFunction(pf)) {
        cout << "Controller can't create potential function!!" << endl;
        return;
    }

    cout << "Controller " << inputSpace.c_str() << " -> " << outputSpace.c_str() << endl;
    if(outputSpace != inputSpace) {
        needsJacobian = true;
        if(!createJacobian()) {
            cout << "Controller can't create Jacobian!!" << endl;
            return;
        }    
    }

    if(!needsJacobian) {
        Vout.resize(potentialFunction->getInputSize());
    } 

    actionName = "/cb/controller/";
    actionName += "s:" + sensorName + "/";
    actionName += "e:" + effectorName + "/";
    actionName += "pf:" + pfName;
}


bool CB::Controller::createPotentialFunction(string pf) {

    bool ok = true;
    vector<string> pfInputs;

    if(!distributedMode) {
        cout << "Controller can't create potential function in local mode..." << endl;
        ok = false;
    }

    pfInputs.clear();
    pfInputs.push_back(sensorName);
    if(hasReference) pfInputs.push_back(referenceName);

    potentialFunction = PotentialFunctionFactory::instance().createObject(pf,pfInputs);
    if(potentialFunction==NULL) {
        ok = false;
    } else {
        inputSpace = potentialFunction->getSpace();
    }

    return ok;
}


bool CB::Controller::createJacobian() {

    bool ok = true;

    if(!needsJacobian) {
        cout << "Controller doesnt need jacobian!!" << endl;
        ok = false;
        return ok;
    }

    jacobian = JacobianFactory::instance().createObject(inputSpace,outputSpace,deviceName);
    needsJacobianInverse = JacobianFactory::instance().needsInverse(inputSpace,outputSpace);

//    jacobian = jacMap.getJacobian(inputSpace,outputSpace,deviceName);
//    needsJacobianInverse = jacMap.needsInverse(inputSpace,outputSpace);

    if(needsJacobianInverse) {
        cout << "Controller needs INVERSE jacobian" << endl;
    } else {
        cout << "Controller needs  jacobian" << endl;
    }

    if(jacobian==NULL) {
        cout << "Controller needs an unknown Jacobian..." << endl;
        ok = false;
        return ok;
    }

    return ok;
}


void CB::Controller::parseOutputResource() {

    string cb = "/cb/";
    deviceName = "";
    string res = effectorName;

    res.erase(0,cb.size());
    string s = "";
    outputSpace = "";
    int i=0;

    s = res[i];
    while(s != "/") {
        outputSpace += s;
        s = res[++i];
    }
    for(;i<res.size();i++) {
        deviceName += res[i];
    }
    cout << "Controller found output space = " << outputSpace.c_str() << ", device=" << deviceName.c_str() << endl;

}


bool CB::Controller::updateAction() {

    // gets the potential information (potential, gradient, type)
    // if needs a jacobian (or inverse), multiplies it here
    // computes -J^# \sigma(\tau)
    // stores (\phi, \dot{\phi})
    // sets state of controller

    Vector grad = potentialFunction->getPotentialGradient();
    potential = potentialFunction->getPotential();

    Matrix J(1,grad.size());
    Matrix JT(grad.size(),1);
    Matrix JTinv(1,grad.size());
    Matrix Jinv(grad.size(),1);
    Matrix Jint(1,1);
        
    // copy the gradient of the pf into the jacobian (and its transpose), and take inverses.
    for(int i=0; i<grad.size(); i++) {
        J[0][i] = grad[i];
        JT[i][0] = grad[i];
    }
    JTinv = pinv(JT,0.0);

    for(int i=0; i<grad.size(); i++) {
        Jinv[i][0] = JTinv[0][i];
    }

    // if there needs to be an intermediate jacobain computed, connect to it here.
    if(needsJacobian) {

        if(!needsJacobianInverse) {
            Jint.resize(jacobian->getOutputSize(),jacobian->getInputSize());
            Jint = jacobian->getJacobian();
        } else {
            Jint.resize(jacobian->getInputSize(),jacobian->getOutputSize());

            if(useJacobianTranspose) {
                Jint = jacobian->getJacobianTranspose();            
            } else {
                Jint = jacobian->getJacobianInverse();            
            }

        }

        if(Vout.size() != Jint.rows())
            Vout.resize(Jint.rows());

        Matrix Jfull(Vout.size(),1);
        Matrix JfullT;
        Matrix JfullInv;
        Matrix JfullInvT;

        // copy the full jacobian based on whether we are using transpose or inverse mode
        if(!useJacobianTranspose) {
            Jfull = Jint*Jinv;
        } else {
            Jfull = Jint*JT;
        }

        // compute the controlle output
        for(int i=0; i<Vout.size(); i++) {
            //Vout[i] = Jfull[i][0]*(-gain*potential - 2.0*gain*potentialDot);
            Vout[i] = -gain*potential*Jfull[i][0];
        }

        if(useJacobianTranspose) {
            Jc = Jfull.transposed();
        } else {
            Jc = pinv(Jfull,0.0);
        }
        
    } else {
        
        // make sure sizes are consistent
        if(Vout.size() != grad.size()) 
            Vout.resize(grad.size());
        
        for(int i=0; i<Vout.size(); i++) {
            if(useJacobianTranspose) {
                Vout[i] = -potential*JT[0][i]*gain;
            } else {
                Vout[i] = -potential*Jinv[0][i]*gain;
            }
        }
        
        Jc = J;
    }


    // estimate the change in potential and store it
    double pdiff = potential - potentialLast;
    double a = 0.2;
    potentialDot = a*pdiff + (1.0-a)*potentialDot;
    potentialLast = potential;
    //cout << "Controller potential: " << potential << ", potentialDot: " << potentialDot << endl;

    potentialStore.push_back(potential);
    potentialDotStore.push_back(potentialDot);

    // set the state based on the estimated change in potential

    if((fabs(potentialDot) < 1E-6) && (iterations>=20)) {
        dynamicState = CONVERGED;
    } else {
        dynamicState = UNCONVERGED;
    }
 
    return true;
}

void CB::Controller::startAction() {

    cout << "Controller::startAction()" << endl;

    // if in local mode, make sure resources are running
    if(!distributedMode) {
    
        // check to make sure everything is running?
        if(!sensor->isResourceRunning()) {
            cout << "Controller starting sensor..." << endl;
            sensor->startResource();
        }
        if(hasReference) {
            if(!reference->isResourceRunning()) {
                cout << "Controller starting reference..." << endl;
                reference->startResource();
            }
        }
        Time::delay(0.2);
    }

    if(needsJacobian) {
        if(!jacobian->isJacobianRunning()) {
            cout << "Controller starting Jacobian..." << endl;
            jacobian->startJacobian();
        }
    }

    if(!potentialFunction->isPotentialRunning()) {
        cout << "Controller starting potential function..." << endl;
        potentialFunction->startPotentialFunction();
        Time::delay(0.5);
    } else {
        cout << "Controller already running potential function..." << endl;
    }
    running = true;
    start();     // mandatory start function
}

void CB::Controller::stopAction() {

    cout << "Controller::stop() -- stopping thread" << endl;
    stop();     // mandatory stop function

    // clear data
    potentialDotStore.clear();
    potentialStore.clear();
    dynamicState = UNKNOWN;
    iterations=0;

    if(!running) return;

    running = false;

    // stop resources
    if(!distributedMode) {
        sensor->stopResource();
        if(hasReference) {
            reference->stopResource();
        }
        effector->stopResource();
    }

    if(needsJacobian) {
        if(jacobian->isJacobianRunning()) {
            jacobian->stopJacobian();
        }
    }

    if(potentialFunction->isPotentialRunning()) {
        potentialFunction->stopPotentialFunction();
    }

    cout << "Controller::stop() -- finished" << endl;

}

void CB::Controller::resetController() {

    cout << "Controller::resetController()" << endl;

    stopAction();
    cout << "Controller::resetController() -- actions stopped" << endl;

    if(!distributedMode) {
        if(sensor!=NULL) delete sensor; sensor=NULL;
        if(effector!=NULL) delete effector; effector=NULL;
        if(reference!=NULL) delete reference; reference=NULL;
        cout << "Controller::resetController() -- resources deleted" << endl;
    }

    if(jacobian!=NULL) {
        jacobian->stopJacobian();
        delete jacobian; jacobian=NULL;
    }
    cout << "Controller::resetController() -- jacobian deleted" << endl;

    if(potentialFunction!=NULL) {
        potentialFunction->stopPotentialFunction();
        delete potentialFunction; potentialFunction=NULL;
    }
    cout << "Controller::resetController() -- potentialFunction deleted" << endl;

    Vout.clear();    
    reset();

    cout << "Controller::resetController() -- done" << endl;
}


void CB::Controller::postData() 
{
    
    Bottle &b = outputPorts[0]->prepare();
    b.clear();                            
    b.addInt(Vout.size());
    
    cout << "Vout: (";
    for(int k = 0; k < Vout.size(); k++) {
        cout << Vout[k] << " ";
        
        // add position to output port
        b.addDouble(Vout[k]);
    }
    cout << endl;
    outputPorts[0]->write();      
}



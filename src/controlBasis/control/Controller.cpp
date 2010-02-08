// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "Controller.h"
#include <yarp/os/Network.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

CB::Controller::Controller(ControlBasisResource *sen,
                           ControlBasisResource *ref,
                           ControlBasisPotentialFunction *pf,
                           ControlBasisResource *eff) {
    
    string refType = "";
    distributedMode = false;    
    sensor = sen;
    effector = eff;
    potentialFunction = pf;    
    potentialDot = 0;
    potentialLast = 0;
    iterations = 0;
    dynamicState = UNKNOWN;
    running = false;
    needsJacobian = false;
    needsJacobianInverse = false;
    hasReference = false;   
    jacobian = NULL;
    potentialFunction = NULL;
    gain = 1.0;
    useJacobianTranspose = true;
    
    // get the types of the resources
    inputSpace = sensor->getResourceType();
    outputSpace = effector->getResourceType();
    pfType = potentialFunction->getInputSpace();
    
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
    outputName.push_back("u");
    
    int inputSize = sensor->getResourceDataSize();
    int outputSize = effector->getResourceDataSize();
    Vout.resize(outputSize);
    
}

CB::Controller::Controller(ControlBasisResource *sen,
                           ControlBasisPotentialFunction *pf,
                           ControlBasisResource *eff) {
    
    string refType = "";
    
    sensor = sen;
    effector = eff;
    potentialFunction = pf;
    potentialDot = 0;
    potentialLast = 0;    
    iterations = 0;
    dynamicState = UNKNOWN;
    running = false;
    needsJacobian = false;
    needsJacobianInverse = false;
    hasReference = false;
    distributedMode = false;    
    jacobian = NULL;
    potentialFunction = NULL;
    useJacobianTranspose = true;
    gain = 1.0;

    // get the types of the resources
    inputSpace = sensor->getResourceType();
    outputSpace = effector->getResourceType();
    pfType = potentialFunction->getInputSpace();
    
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
    outputName.push_back("u");
    
    int inputSize = sensor->getResourceDataSize();
    int outputSize = effector->getResourceDataSize();
    Vout.resize(outputSize);
    
}

CB::Controller::Controller(string sen, string ref, string pf, string eff) {

    sensorName = sen;
    referenceName = ref;
    effectorName = eff;
    pfName = pf;
    distributedMode = true;
    needsJacobian = false;
    needsJacobianInverse = false;
    hasReference = true;
    gain = 1.0;
    dynamicState = UNKNOWN;
    potentialDot = 0;
    potentialLast = 0;
    jacobian = NULL;
    potentialFunction = NULL;
    useJacobianTranspose = true;

    if(!connectToResources(sen,ref,eff)) {
        cout << "Controller can't connect to resources!!" << endl;
        return;
    }
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

CB::Controller::Controller(string sen, string pf, string eff) {

    sensorName = sen;
    referenceName = "";
    effectorName = eff;
    pfName = pf;
    distributedMode = true;
    needsJacobian = false;
    needsJacobianInverse = false;
    hasReference = false;
    gain = 1.0;
    dynamicState = UNKNOWN;
    potentialDot = 0;
    potentialLast = 0;
    jacobian = NULL;
    potentialFunction = NULL;
    useJacobianTranspose = true;

    if(!connectToResources(sen,"",eff)) {
        cout << "Controller can't connect to resources!!" << endl;
        return;
    }
    if(!createPotentialFunction(pf)) {
        cout << "Controller can't create potential function!!" << endl;
        return;
    }

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

    cout << "Controller needs jacobian: " << needsJacobian << endl;

    actionName = "/cb/controller/";
    actionName += "s:" + sensorName + "/";
    actionName += "e:" + effectorName + "/";
    actionName += "pf:" + pfName;
}


bool CB::Controller::connectToResources(string sen, string ref, string eff) {

    bool ok = true;

    int randomID;
    Random::seed(time(NULL));
    randomID = (int)(Random::uniform()*1000.0);
    char *c;
    c = (char *)malloc(16);
    sprintf(c, "%d", randomID);
    string randomIDstr(c);

    sensorOutputName = sen + "/data:o";
    sensorInputName = "/cb/controller/" + randomIDstr + "/sensor" + sen + "/data:i";
    ok &= sensorInputPort.open(sensorInputName.c_str());
    ok &= Network::connect(sensorOutputName.c_str(),sensorInputName.c_str(),"tcp");
    if(!ok) {
        cout << "Controller couldn't connect to sensor: " << sensorOutputName.c_str() << endl;
        return ok;
    }
    cout << "Controller connected sensor." << endl;

    if(hasReference) {
        refOutputName = ref + "/data:o";
        refInputName = "/cb/controller/" + randomIDstr + "/ref" + ref + "/data:i";
        ok &= refInputPort.open(refInputName.c_str());
        ok &= Network::connect(refOutputName.c_str(),refInputName.c_str(),"tcp");
        if(!ok) {
            cout << "Controller couldn't connect to reference: " << refOutputName.c_str() << endl;
            return ok;
        }
        cout << "Controller connected reference." << endl;
    }

    effectorOutputName = eff + "/data:o";
    effectorInputName = "/cb/controller/" + randomIDstr + "/effector" + eff + "/data:i";

    ok &= effectorInputPort.open(effectorInputName.c_str());
    ok &= Network::connect(effectorOutputName.c_str(),effectorInputName.c_str(),"tcp");
    if(!ok) {
        cout << "Controller couldn't connect to effector: " << effectorOutputName.c_str() << endl;
        return ok;
    }
    cout << "Controller connected effector." << endl;

    // get effector type
    parseOutputResource();    

    return ok;

}

bool CB::Controller::createPotentialFunction(string pf) {

    bool ok = true;

    if(!distributedMode) {
        cout << "Controller can't create potential function in local mode..." << endl;
        ok = false;
    }


    if(hasReference) {
        potentialFunction = pfMap.getPotentialFunction(pf,sensorName,referenceName);
    } else {
        potentialFunction = pfMap.getPotentialFunction(pf,sensorName,"");
    }

    if(potentialFunction==NULL) {
        ok = false;
    } else {
        inputSpace = potentialFunction->getInputSpace();
        cout << "Controller found input type = " << inputSpace.c_str() << endl;
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

    jacobian = jacMap.getJacobian(inputSpace,outputSpace,deviceName);
    needsJacobianInverse = jacMap.needsInverse(inputSpace,outputSpace);

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

    //    cout << "CONTROLLER REQUESTING POTENTIAL\n" << endl;
    Vector grad = potentialFunction->getPotentialGradient();
    potential = potentialFunction->getPotential();

    Matrix J(1,grad.size());
    Matrix JT(grad.size(),1);
    Matrix JTinv(1,grad.size());
    Matrix Jinv(grad.size(),1);
    Matrix Jint(1,1);

        
    //    cout << "Controller gradient:" << endl;
    for(int i=0; i<grad.size(); i++) {
        J[0][i] = grad[i];
        JT[i][0] = grad[i];
        //  cout << J[0][i] << endl;
    }
    JTinv = pinv(JT,0.0);

    //    cout << "Controller gradient INVERSE:" << endl;
    for(int i=0; i<grad.size(); i++) {
        Jinv[i][0] = JTinv[0][i];
        //  cout << Jinv[i][0] << endl;
    }

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

          /*
            cout << "\nController Jint:\n";
            for(int row=0; row<Jint.rows(); row++) {
                for(int col=0; col<Jint.cols(); col++) {
                    cout << Jint[row][col] << " ";
                }
                cout << endl;
            }
          */
        }

        if(Vout.size() != Jint.rows())
            Vout.resize(Jint.rows());

        Matrix Jfull(Vout.size(),1);
        Matrix JfullT;
        Matrix JfullInv;
        Matrix JfullInvT;

        if(!useJacobianTranspose) {
            Jfull = Jint*Jinv;
        } else {
            Jfull = Jint*JT;
        }

        /*
        cout << "\nController Jfull:\n";
        for(int row=0; row<Jfull.rows(); row++) {
            for(int col=0; col<Jfull.cols(); col++) {
                cout << Jfull[row][col] << " ";
            }
            cout << endl;
        }
        cout << endl;
        */

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

    
    /*
    cout << "Controller[" << actionName.c_str() << "] output:" << endl;
    for(int i=0; i<Vout.size(); i++) {
        cout << Vout[i] << endl;
    }
    */
    
    double pdiff = potential - potentialLast;
    double a = 0.2;
    potentialDot = a*pdiff + (1.0-a)*potentialDot;
    potentialLast = potential;
    //cout << "Controller potential: " << potential << ", potentialDot: " << potentialDot << endl;

    potentialStore.push_back(potential);
    potentialDotStore.push_back(potentialDot);

    if(fabs(potentialDot) < 1E-5) {
        dynamicState = CONVERGED;
    } else {
        dynamicState = UNCONVERGED;
    }
 
    return true;
}

void CB::Controller::startAction() {

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
        Time::delay(0.5);
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

    cout << "Controller::stop() -- killing jacobian and pf" << endl;
    if(jacobian!=NULL) jacobian->stopJacobian();
    if(potentialFunction!=NULL) potentialFunction->stopPotentialFunction();

    // close ports
    if(distributedMode) {
        cout << "Controller::stop() -- closing sensor input port" << endl;
        sensorInputPort.close();
        if(hasReference) {
        cout << "Controller::stop() -- closing reference input port" << endl;
            refInputPort.close();
        }
        cout << "Controller::stop() -- closing effector input port" << endl;
        effectorInputPort.close();
    } else {
        sensor->stopResource();
        if(hasReference) {
            reference->stopResource();
        }
        effector->stopResource();
    }


    potentialDotStore.clear();
    potentialStore.clear();

    cout << "Controller::stop() -- ports closed" << endl;
}

void CB::Controller::postData() {
    
    Bottle &b = outputPort[0]->prepare();
    b.clear();
    
    cout << "Controller " << actionName.c_str() << " posting data" << endl;
  
    b.addInt(Vout.size());
    
    cout << "out: (";
    for(int i = 0; i < Vout.size(); i++) {
        cout << Vout[i] << " ";
       
        // add position to output port
        b.addDouble(Vout[i]);
    }
    cout << endl;
    outputPort[0]->write();      
 
}

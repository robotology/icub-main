// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "Schema.h"
#include <yarp/os/Network.h>
#include <yarp/math/Math.h>

#include <combinatorics.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

CB::Schema::Schema(vector<ControllerParameters> controllerParams, vector<string> subSchema, int limit, int nStatePredicates, int nPredicateValues) :
    ControlBasisAction(),
    controllerParameters(controllerParams),
    schemaList(subSchema),
    compositeLimit(limit),
    value(0),
    valueLast(0),
    valueDot(0),
    internalState(0),
    internalStateLast(-1),
    action(0),
    actionLast(-1),
    cumulativeReward(0),
    explore(true),
    numPrimitiveControllers(0),
    numCompositeControllers(0),
    numSchema(0),
    numActions(0),
    numStates(0),
    numStatePredicates(nStatePredicates),
    numPredicateValues(nPredicateValues),
    schemaName("schema")
{

    numPrimitiveControllers = controllerParameters.size();
    numSchema = schemaList.size();

    actionMask.clear();
    if(compositeLimit > numPrimitiveControllers) {
        compositeLimit=numPrimitiveControllers;
    }
    
    if(compositeLimit<1) {
        compositeLimit=1;  
    }

    int c=0;
    for(int i=1; i<=compositeLimit; i++) {
        c  += permutations(numPrimitiveControllers, i);
    }    
    for(int i=0; i<c; i++) {
        actionMask.push_back(getPermutationMask(numPrimitiveControllers,i));
    }
    numCompositeControllers=c;
    c += numSchema;

    numCompositeControllers -= numPrimitiveControllers;
    numStates = pow(numPredicateValues,numStatePredicates);
    numActions = c;

    mdp = new MarkovDecisionProcess(numStates,numActions); //add value file as third parameter

    string iCubDir(getenv("ICUB_ROOT"));
    string vfile = iCubDir + "/app/controlBasis/conf/" + schemaName + ".xml";
    mdp->setValueFile(vfile);
    cout << "setting value function name: " << vfile << endl; 

    cout << endl;
    cout << "GENERALIZABLE SCHEMA INFO:" << endl;
    cout << "------------------------------" << endl;
    cout << "composite lim : " << compositeLimit << endl;
    cout << "num pred vals : " << numPredicateValues << endl;
    cout << "num states    : " << numStates << endl;    
    cout << "num primitives: " << numPrimitiveControllers << endl;
    cout << "num composite : " << numCompositeControllers << endl;
    cout << "num schema    : " << numSchema << endl;
    cout << "total actions : " << numActions << endl << endl;    

    controllers.clear();
    for(int i=0; i<numPrimitiveControllers; i++) {
        controllerParameters[i].display();

        if(controllerParameters[i].Reference != "") {
            controllers.push_back(new Controller(controllerParameters[i].Sensor,
                                                 controllerParameters[i].Reference,
                                                 controllerParameters[i].PotentialFunction,
                                                 controllerParameters[i].Effector));
        } else {            
            controllers.push_back(new Controller(controllerParameters[i].Sensor,
                                                 controllerParameters[i].PotentialFunction,
                                                 controllerParameters[i].Effector));
        }
        controllers[i]->setGain(controllerParameters[i].Gain);
    }
    
}


bool CB::Schema::updateAction() {

    double reward;
    double updateIterations;
    string actionStr;
    int action_id;

    cout << endl << endl << "SCHEMA[" << schemaName << "] UPDATE, iteration: " << iterations << endl << "--------------------------------------------" << endl;

    // get the best action for the given state 
    action = mdp->getMaxAction(internalState, explore);
    cout << "MDP recommends action: " << action << endl;

    // technically we should only check to see if the action is different from the actionLast.
    // if it is, then we should stop the existing control law, and start the new one.
    // However, in the case when the action is the same, and the action has converged,
    // no state change will happen.  If we restart the action, even after its converged,
    // it will first go to uncoverged (albeit temporarily), before it converges again (quickly).
    // The other option would be to only restart if converged, or not restart if uncoverged.
    // I suspect this will be a problem for undefined controllers.  for example, transitioning
    // from state XX to X- for action #2, will cause a state change, but re-selecting action #2
    // from this state will not.  In contrast, going from XX to X0 to X1 will, and so will 
    // X0 to X1 to X0 to X1 (and so on).  maybe there should be a time out? hmmmm...
    // it gets complicated with nullspace combinations: which controllers do we test?
    // all of them? just the highest priority one? and so on....

    // set up the action to be run
    if(action < (numPrimitiveControllers+numCompositeControllers)) {
        
        // reset the control law in case it is running
        controlLaw.resetControlLaw();  
        
        // find the the string mask of actions to run
        actionStr = actionMask[action];
        cout << "Schema found action string: " << actionStr << endl;        

        // go through string and add the controllers to the control law in the order specified
        for(int i=0; i<actionStr.size(); i++) {
            action_id = actionStr[i]-48; // go from ASCII to Integer ('0'==48)
            controlLaw.addController(controllers[action_id]);
        }
        controlLaw.useTranspose(true);
        
    } else {
        
        // if it is a schema called hierarchically
        schema_id = action - (numPrimitiveControllers + numCompositeControllers);
        cout << "schema choose sub schema[" << schema_id << "] to run..." << endl;        
        
        schema[schema_id]->resetSchema(); // ?
        
    }

    // evaluate whether we are at goal.
    // Set to converged if so, and break out...
    if(mdp->isGoalState(internalState)) {
        cout << "Schema reached goal state[" << internalState << "]" << endl;
        dynamicState = CONVERGED;
        stopAction();
        save();
        return true;
    } 

    // start the action chosen
    if(action < (numPrimitiveControllers+numCompositeControllers)) {
        cout << "schema starting control law..." << endl;
        controlLaw.startAction();
        Time::delay(updateDelay);
    } else {
        cout << "schema starting sub-schema..." << endl;
        schema_id = action - (numPrimitiveControllers + numCompositeControllers);
        schema[schema_id]->startAction();
    }

    // store the last action
    actionLast = action;

    // store the state
    internalStateLast = internalState;
    internalState = evaluateState();
    updateIterations=0;


    cout << "Schema starting update loop (lastState=" << internalStateLast << ", state=" << internalState << ")" << endl;

    // iterate on the action until a state change (timeout on updateIterations?)
    while(internalState == internalStateLast) {
        internalState = evaluateState();       
        updateIterations += 1.0;
        Time::delay(0.5);
    }

    // evaluate the reward
    reward = evaluateReward(internalState, internalStateLast, action, actionLast);
    cumulativeReward += reward;
    
    // update the MDP with the new information
    mdp->updateRule(internalStateLast, action, reward, internalState, MarkovDecisionProcess::Q_LEARNING);
    
    // need to update the value and value_dot variables
    valueLast=value;    
    value=0;
    for(int a=0; a<numActions; a++) {
        value += mdp->getValue(internalState,a);
    }
    value /= numActions;
    valueDot = (value-valueLast)/updateIterations;

    // update schema iteration count
    iterations++;

    // evaluate whether we are at goal, and set to converged if so
    if(mdp->isGoalState(internalState)) {
        dynamicState = CONVERGED;
    }

    /*
    // set to converged if change in value is low enough....
    if(valueDot < 1E-10) {
        dynamicState = CONVERGED;
    }  
    */

    mdp->updateRule(internalStateLast, action, reward, internalState, MarkovDecisionProcess::Q_LEARNING);
    mdp->updateTransitionProbabilities(internalStateLast, action, internalState);

    return true;
}

void CB::Schema::startAction() {
    cout << "Schema::startAction()" << endl;
    running = true;    
    dynamicState = UNCONVERGED;
    start();     // mandatory start function   
}

void CB::Schema::stopAction() {

    if(isRunning()) {

        cout << "Schema::stop() -- stopping thread" << endl;
        stop();     // mandatory stop function    
        
        //        while(isStopping()) {
        //  Time::delay(0.1);
        //  cout << "waiting for schema to stop..." << endl;
        // }
        cout << "Schema::stop() -- stopping control law" << endl;
        controlLaw.stopAction();

    }
    dynamicState = UNKNOWN;

}

void CB::Schema::resetSchema() {
    cout << "Schema::resetSchema()" << endl;
    stopAction();
    reset();
    cout << "Schema::resetSchema() -- done" << endl;
}

void CB::Schema::postData() 
{   
}


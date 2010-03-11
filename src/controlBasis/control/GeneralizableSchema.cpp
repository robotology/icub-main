// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "GeneralizableSchema.h"

using namespace std;

CB::GeneralizableSchema::GeneralizableSchema(vector<ControllerParameters> controllerParams, vector<string> subSchema, int limit) 
    : Schema(controllerParams, subSchema, limit, (controllerParams.size() + subSchema.size()), 4)
{
}

CB::GeneralizableSchema::~GeneralizableSchema() {
}

double CB::GeneralizableSchema::evaluateReward(int state, int last_state, int action, int last_action) {
    double r=0;
    
    if(mdp->isGoalState(state) && !(mdp->isGoalState(last_state))) {
        r = 1;
    }

    return r;
}
 
int CB::GeneralizableSchema::evaluateState() {

    internalState = 0;
    int s;
    int numPredicates = numPrimitiveControllers+numSchema;
    for(int i=0; i<numPrimitiveControllers; i++) {
        //                cout << "getting state from controller["<<i<<"]";
        s = controllers[i]->getState();   
        cout << "controller[" << i << "] p: " << controllers[i]->getControllerPotential() << ", pdot: " << controllers[i]->getControllerPotentialDot() << endl;
        //cout << ": " << s << endl;             
        internalState += s*(int)pow((double)numPredicateValues,(double)(numPredicates - i - 1));    
    }
    for(int i=0; i<numSchema; i++) {
        s = schema[i]->getState();    
        internalState += s*(int)pow((double)numPredicateValues,(double)(numPredicates - (i+numPrimitiveControllers) - 1));    
    }
    
    mdp->setState(internalState);
    
    internalStateString = getStringFromState(internalState);
    
    return internalState;
}

int CB::GeneralizableSchema::getStateFromString(string str) {
    int tmp=0;
    int s;
    for(int i=0; i<str.length(); i++) {
        if(str[i]=='X') {
            s = UNKNOWN;
        } else if(str[i]=='-') {
            s = UNDEFINED;
        } else if(str[i]=='0') {
            s = UNCONVERGED;
        } else if(str[i]=='1') {
            s = CONVERGED;
        }  else {
            cout << "Schema unknown state!! -- " << str << endl;
            return -1;
        }
        tmp += s*(int)pow((double)numPredicateValues,(double)(numStatePredicates - i - 1));    
    }
    return tmp;
}

string CB::GeneralizableSchema::getStringFromState(int s) {

    int k ,c;
    string tmp = "";
    int s_init = s;

    // figure out the string representation (base 4 where the vals=[X,-,0,1])
    k = 0;
    while(k<(numPrimitiveControllers+numSchema)) {
        c = s%numPredicateValues;
        switch(c) {
        case UNKNOWN:  
            tmp = "X" + tmp;;
            break;
        case UNDEFINED:
            tmp = "-" + tmp;;
            break;
        case UNCONVERGED:
            tmp = "0" + tmp;;
            break;
        case CONVERGED:
            tmp =  "1" + tmp;;
            break;
        default:
            break;
        }
        s /= 4;
        k++;
    }
    

    // print it out
    cout << "Schema[" << schemaName << "], state[" << s_init << "]: " << tmp << endl;
    
    return tmp;
}


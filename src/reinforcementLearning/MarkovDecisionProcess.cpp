// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iostream>
#include <math.h>
#include <tinyxml.h>

#include "MarkovDecisionProcess.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;

MarkovDecisionProcess::MarkovDecisionProcess(int numStates, int numActions, string valueFile) :
    _numStates(numStates),
    _numActions(numActions)
{

    //  _initValue = 1.0/_numActions;
    _initValue = 0.0;
    _valueFile = valueFile;
    _policy.clear();

    _policy.resize(_numStates);
    for(int i=0; i<_numStates; i++) {
        _policy[i]=-1;
    }

    if(valueFile!="") {
        _valueFileSet = true;
        if(!loadValueFunction()) {
            cout << "Error loading MDP Value Function: " << valueFile.c_str() << endl;
            exit(0);
        }           
    } else {
        _valueFileSet = false;
        if( (_numStates == 0) && (_numActions == 0) ) {
            cout << "Error Creating MDP.  Needs num states and actions and/or value file!!" << endl;
            exit(0);
        }
    }


    setState(0);
    Random::seed(time(NULL));

    cout << "MDP: Created "<< _numStates <<" states, " << _numActions << " actions" << endl;       

}

MarkovDecisionProcess::~MarkovDecisionProcess() { }


void MarkovDecisionProcess::setState(int s) { 
    _last_state = _state;
    _state = s;
}

double MarkovDecisionProcess::getValue(int s, int a) {
    int id = _numStates*s+a;
    if(_values.find(id) == _values.end()) {
        return _initValue;
    } else {
        return _values[id];
    }
}

void MarkovDecisionProcess::setValue(int s, int a, double v) {
    int id = _numStates*s+a;  
    if( fabs(v-_initValue) > SMALL_VAL ) { 
        _values[id] = v;
    }
}
  
double MarkovDecisionProcess::getEntropy(int s, int a) {
    int id = _numStates*s+a;
    if(_entropy.find(id) == _entropy.end()) {
        return 0.0;
    } else {
        return _entropy[id];
    }
}
  
void MarkovDecisionProcess::setEntropy(int s, int a, double v) {
    int id = _numStates*s+a;
    if( fabs(v) > SMALL_VAL ) { 
        _entropy[id] = v;
    }
}

double MarkovDecisionProcess::getTransitionProb(int s, int a, int sp) {
    int id = (_numActions*_numStates)*s+(_numStates)*a+sp;
    if(_transitions.find(id) == _transitions.end()) {
        return 0.0;
    } else {
        return _transitions[id];
    }
}

void MarkovDecisionProcess::setTransitionProb(int s, int a, int sp, double v) {
    int id = (_numActions*_numStates)*s+(_numStates)*a+sp;
    if( fabs(v) > SMALL_VAL ) { 
        _transitions[id] = v;
    }
}

int MarkovDecisionProcess::getTransitions(int s, int a, int sp) {
    int id = (_numActions*_numStates)*s+(_numStates)*a+sp;
    if(_numTransitions.find(id) == _numTransitions.end()) {
        return 0;
    } else {
        return _numTransitions[id];
    }
}

void MarkovDecisionProcess::setTransitions(int s, int a, int sp, int n) {
    int id = (_numActions*_numStates)*s+(_numStates)*a+sp;
    if( n!=0 ) { 
        _numTransitions[id] = n;
    }
}

double MarkovDecisionProcess::getEligibility(int s, int a) {
    int id = _numStates*s+a;
    if(_eligibility.find(id) == _eligibility.end()) {
        return 0.0;
    } else {
        return _eligibility[id];
    }
}

void MarkovDecisionProcess::setEligibility(int s, int a, double v) {
    int id = _numStates*s+a;
    if( fabs(v) > SMALL_VAL ) { 
        _eligibility[id] = v;
    }
}

void MarkovDecisionProcess::addGoalState(int s) {

    bool found=false;

    // check to see we don't already have it...
    for(int i=0; i<_goalStates.size(); i++) {
        if(_goalStates[i]==s) found = true;
    }
        
    if(!found) {
        _goalStates.push_back(s);
    }
    

}

int MarkovDecisionProcess::isGoalState(int s) {
    int r=0;
    for(int i=0; i<_goalStates.size(); i++) {
        if(_goalStates[i]==s) {
            r=1;
            break;
        }
    }
    return r;
}

int MarkovDecisionProcess::loadValueFunction() {

    int nStates, nActions;
    int best_action;
    int user_specified_action;
    int state = 0;
    int next_state = 0;
    int action = 0;
    int nt;
    double action_val;
    int goal;
    double r, max_q, qval;
    
    
    if(!_valueFileSet) {
        cout << "MDP::loadValueFunction() -- no value function file set!!" << endl;
        return 0;
    }

    cout << "loading xml file..." << endl;

    TiXmlDocument doc( _valueFile.c_str() );

    if(!doc.LoadFile()) {
        cout << "MDP::loadValueFunction() -- failed to load xml file..." << endl;
        return 0;
    }
    
    TiXmlHandle hDoc(&doc);
    TiXmlHandle hRoot(0);

    TiXmlElement *pElem;
    TiXmlElement *pAction;
    TiXmlElement *pNextState;

    const char *pSpecified;
    const char *pValue;

    pElem=hDoc.FirstChildElement().Element();
    if(!pElem) return 0;
    string m_root=pElem->Value();

    if(m_root!="QVALUES") {
        cout << "MDP::loadValueFunction() -- failed to find QValues root..." << endl;
        return 0;
    }
    hRoot=TiXmlHandle(pElem);

    // get number of states and number of actions
    pElem->QueryIntAttribute("NUM-STATES", &nStates);
    pElem->QueryIntAttribute("NUM-ACTIONS", &nActions);
    cout << "MDP: Found "<< nStates <<" states, " << nActions << " actions" << endl;       

    if( (_numStates != 0) && (_numActions != 0) ) {
        if( (nStates != _numStates) || (nActions != _numActions) ) {
            cout << "This does not agree with existing specification of " << _numStates << " states, " << _numActions << " actions" << endl;       
            return 0;
        }   
    } else {

        // set global vars to what is read from the file
        _numStates = nStates;
        _numActions = nActions;

        if(_policy.size() == 0) {
            _policy.resize(_numStates);
            for(int i=0; i<_numStates; i++) {
                _policy[i]=-1;
            }
        }

    }

    pElem=hRoot.FirstChild().Element();
    for( pElem; pElem; pElem=pElem->NextSiblingElement() ) {
        pValue = pElem->Value();
       
        if(!strncmp(pValue,"GOAL",4)) {
        
            // get any goals
            pElem->QueryIntAttribute("ID", &goal);
            cout << "Found Goal: " << goal << endl;       
            addGoalState(goal);
   
        } else if(!strncmp(pValue,"REWARD",6)) {

            // get reward statistics
            pElem->QueryDoubleAttribute("VAL", &r);
            pElem->QueryDoubleAttribute("MAX_Q", &max_q);
            _rewards.push_back(r);
            _maxVals.push_back(max_q);
            //cout << "Found Reward Statistic: (r=" << r << ", q=" << max_q << ")" << endl;       
        
        } else if(!strncmp(pValue,"STATE",5)) {

            // get state/action information
            pElem->QueryIntAttribute("ID", &state);
            pElem->QueryIntAttribute("BEST-ACTION", &best_action);            
            //            cout << "State[" << state << "]: action=" << best_action;          
            if((pElem->Attribute("USER-SPECIFIED"))!=NULL) {
                pElem->QueryIntAttribute("USER-SPECIFIED", &user_specified_action);
                _policy[state] = best_action;
                //cout << ", user specified";                
            } else {
                _policy[state] = -1;
            }
            //cout << endl;
                        
            // get the possible next states for that action (in that state)
            pAction=pElem->FirstChildElement();
            for( pAction; pAction; pAction=pAction->NextSiblingElement() ) {
                pValue = pAction->Value();            
                if(!strncmp(pValue,"ACTION",6)) {
                    pAction->QueryIntAttribute("ID", &action);
                    pAction->QueryDoubleAttribute("QVAL", &qval);            
                    //cout << "\tAction[" << action << "]: qval=" << qval << endl;       
                    setValue(state,action,qval);

                    pNextState=pAction->FirstChildElement();
                    for( pNextState; pNextState; pNextState=pNextState->NextSiblingElement() ) {
                        pValue = pNextState->Value();            
                        if(!strncmp(pValue,"NEXT-STATE",10)) {
                            pNextState->QueryIntAttribute("ID", &next_state);
                            pNextState->QueryIntAttribute("N", &nt);            
                            //cout << "\t\tNextState[" << next_state << "]: n=" << nt << endl;       
                            setTransitions(state,action,next_state,nt);
                        }
                    }
                }
            }
        }        
    }
   
    // take the number of transitions and compute transition probabilities and entropy
    int c;           
    double e,ep;
    for(int i=0; i<_numStates; i++) {
        for(int j=0; j<_numActions; j++) {
            c = 0;
            // count the total
            for(int k=0; k<_numStates; k++) {
                c += getTransitions(i,j,k);
            }
            // as long as there were some:
            if(c!=0) {
                for(int k=0; k<_numStates; k++) {
                    setTransitionProb(i,j,k, (getTransitions(i,j,k))/(double)c);	  
                    e = (getTransitionProb(i,j,k) * (log((getTransitionProb(i,j,k)))/log(2.0)));
                    if(!isnan(e)) {
                        ep = getEntropy(i,j) - e;
                        setEntropy(i,j,ep);
                    } else {
                        setEntropy(i,j,0.0);
                    }
                    //cout << "MarkovDecisionProcess::loadValueFunction() -- setting transition["<<i<<"]["<<j<<"]["<<k<<"]="<<getTransitionProb(i,j,k)<<endl; 
                }
            }
        }
    }        

    return 1;
}


void MarkovDecisionProcess::writeValueFunction(double reward) {
  
    int a,i,j,k,n;
    int max_action;
    double max_value;
    int c,s,swap;
    int user_specified_action = 0;

    TiXmlDocument doc;
    TiXmlElement *msg;
    TiXmlComment *comment;

    cout << "MDP -- writing value function to file: " << _valueFile.c_str() << endl;

    TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);

    TiXmlElement *root = new TiXmlElement("QVALUES");
    doc.LinkEndChild(root);
    root->SetAttribute("NUM-STATES", _numStates);
    root->SetAttribute("NUM-ACTIONS", _numActions);

    // store reward
    _rewards.push_back(reward);
    _maxVals.push_back(getMaxReward(s,a));

    // print out the goal states
    for(i=0; i<_goalStates.size(); i++) {
        TiXmlElement *gElem = new TiXmlElement("GOAL");
        root->LinkEndChild(gElem);
        gElem->SetAttribute("ID", _goalStates[i]);
    }

    // go through each state and print its information 
    // (including the qvals for each action, and the possible next states observed 
    // after taking those actions)
    for(i=0; i<_numStates; i++) {
    
        max_value = -100000;
        max_action = getMaxAction(i, false);
        //        cout << "state[" << i <<"] got max action: " << max_action << endl;        
        user_specified_action = 0;
        if(_policy[i] != -1) {
            if(max_action != _policy[i]) {
                max_action = _policy[i];
            }
            //	cout << "found a user-specified action for state[" << i << "]: " << _policy[i] << endl;
            user_specified_action = 1;
        }

        // print out the action information for each state
        TiXmlElement *sElem = new TiXmlElement("STATE");
        root->LinkEndChild(sElem);
        sElem->SetAttribute("ID", i);
        sElem->SetAttribute("BEST-ACTION", max_action);
        
        if(user_specified_action==1) {
            sElem->SetAttribute("USER-SPECIFIED", user_specified_action);
        } 

        // print out the qvalue for each state action pair
        for(j=0; j<_numActions; j++) {
            TiXmlElement *aElem = new TiXmlElement("ACTION");
            sElem->LinkEndChild(aElem);
            aElem->SetAttribute("ID", j);
            aElem->SetDoubleAttribute("QVAL", getValue(i,j));

            for(n=0; n<_numStates; n++) {
                // only print if there has been a transition
                if(getTransitions(i,j,n) > 0) {	 
                    TiXmlElement *nElem = new TiXmlElement("NEXT-STATE");
                    aElem->LinkEndChild(nElem);
                    nElem->SetAttribute("ID", n);
                    nElem->SetAttribute("N", getTransitions(i,j,n));
                }
            }
        }      
    }

    // print out the reward information for each episode
    for(i=0; i<_rewards.size(); i++) {
        TiXmlElement *rElem = new TiXmlElement("REWARD");
        root->LinkEndChild(rElem);
        rElem->SetAttribute("EPISODE", i);
        rElem->SetDoubleAttribute("VAL", _rewards[i]);
        rElem->SetDoubleAttribute("MAX_Q", _maxVals[i]);
    }

    cout << endl << "MDP::writeValueFunction() -- writing to file: " << _valueFile.c_str() << endl;
    doc.SaveFile(_valueFile.c_str());

}

int MarkovDecisionProcess::getMaxAction(int s, int explore) {
    
    int a, max_a;
    double v, r;
    double tmp;

    int numExploratoryActions;
    int exploratoryAction;
    double inc;
    double accum;
    double normalizedRandom;

    // check to see if there is a user specified action
    if(_policy[s] != -1) {
        a = _policy[s];    
        //cout << "user specified action for state["<<s<<"]"<<endl;
        return a;
    }    
    
    // find policy optimal action
    a = 0;
    v = getValue(s,a);
    
    for(int i=1; i<_numActions; i++) {     
        tmp = getValue(s,i);
        if(tmp > v) {
            v = tmp;
            a=i;	
        } else if(tmp == v) {
            // if this new action has the same value as the current best, 50% of the time, take the new one
            if(Random::uniform() < 0.5) {
                v = tmp;
                a = i;
            }
        }
    }

    // if we are exploratory, then do e-greedy action selection    
    if(explore) {
        r = Random::uniform();
    } else {
        r = EPSILON;
    }
    
    // choose random action
    if(r < EPSILON) {
        
        max_a = a;
        numExploratoryActions = _numActions - 1;  // all but the optimal
        inc = (1 - (double)EPSILON) / (double)numExploratoryActions;
        accum = 0;
        
        // normalize the number we got
        normalizedRandom = r/(double)EPSILON;
        
        accum += inc;
        exploratoryAction = 0;
        for (int i = 0; i < numExploratoryActions; i++) {
            if (normalizedRandom <= accum) {
                exploratoryAction = i;	               
                break;
            }
            else {
                accum += inc;
            }
        }
        
        // got the exploratory action index, now turn that into the actual action index
        if (exploratoryAction < a)  {
            // if the index is less than a, then the exploratory action ID is the actual action ID
            a = exploratoryAction;                    
        }
        else {
            // if the index is a or greater, then we are off by one (cause we are removing a from the choices)
            a = exploratoryAction+1;
        }

        cout << "MDP picking exploratory action[" << a << "] in state[" << s << "]..." << endl;
        
    }
    
    return a;
  
}


void MarkovDecisionProcess::updateRule(int s, int a, double r, int sp, UpdateRules rule) {
  
    int c = 0;
    double v = getValue(sp,c);
    double val, newVal;
    double ep;
    int p;
    int i;
    double delta;
    int ap;
    
    // find policy optimal action
    for(i=1; i<_numActions; i++) {      
        if(getValue(sp,i) > v) {
            v = getValue(sp,i);
            c = i;
        }
    } 
    
    // Standard Q-Learning update rule
    if( rule == Q_LEARNING ) { 
        
        val = getValue(s,a);
        cout<<"val="<<val<<", r="<<r<<", v="<<v<<endl;
        newVal = val + ALPHA*((double)r + LAMBDA*v - val);     
        setValue(s,a,newVal);
        
    } else if( rule == SARSA_LAMBDA ) { // SARSA(\lambda) with elgibility trace stuff  
        
        if(!isGoalState(sp)) {    
            ap = getMaxAction(sp, 1); // get next (e-greedy action)      
            delta = (double)r + LAMBDA*getValue(sp,ap) - getValue(s,a);
        } else {
            delta = (double)r;
        } 
        ep = getEligibility(s,a) + 1.0;
        setEligibility(s,a,ep);
        for(int i=0; i<_numStates; i++) {
            for(int j=0; j<_numActions; j++) {
                val = getValue(i,j) + ALPHA*delta*getEligibility(i,j);
                setValue(i,j, val);
                ep = GAMMA*LAMBDA*getEligibility(i,j);
                setEligibility(i,j,ep);
            }
        }
        
    }
    
}

void MarkovDecisionProcess::updateTransitionProbabilities(int s, int a, int sp) {
    
    // increase count
    int t =  getTransitions(s,a,sp)+1;
    setTransitions(s,a,sp,t);
    
    // renormalize probabilities
    int c = 0;
    double e,ep;
    // count the total
    for(int k=0; k<_numStates; k++) {
        c += getTransitions(s,a,k);
    }
    // as long as there were some:
    if(c!=0) {
        //      setEntropy(s,a,0.0);
        for(int k=0; k<_numStates; k++) {
            setTransitionProb(s,a,k, (getTransitions(s,a,k))/(double)c);	
            e = (getTransitionProb(s,a,k) * (log(getTransitionProb(s,a,k))/log(2.0)));
            if(!isnan(e)) {
                ep = getEntropy(s,a) - e;
                setEntropy(s,a,ep);
            } else {
                //setEntropy(s,a,0.0);
            }
            
        }
        cout << "MarkovDecisionProcess:: entropy for (s,a)=("<<s<<","<<a<<") is "<<getEntropy(s,a)<<endl;
    }    
    
} 


double MarkovDecisionProcess::getMaxReward(int &s, int &a) {

    double v;
    double max_v = -100000000;
    
    for(int i=0;i<_numStates; i++) {
        for(int j=0;j<_numActions; j++) {
            v = getValue(i,j);
            if(v > max_v) {
                s = i;
                a = j;
                max_v = v;
            }
        }
    }    
    return max_v;    
}

void MarkovDecisionProcess::reset() {

    cout << "MDP -- reset..." << endl;

    _rewards.clear();
    _maxVals.clear();
    _goalStates.clear();
    _policy.clear();
    _policy.resize(_numStates);

    for(int s=0; s<_numStates; s++) {
        _policy[s] = -1;
    }

    _values.clear();
    _entropy.clear();
    _eligibility.clear();
    _transitions.clear();
    _numTransitions.clear();
    
}


// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/**
@ingroup icub_module
\defgroup icub_reinforcementLearning reinforcementLearning

A module that implements a Markov Decision Process (MDP) that can be solved using various (simple)
reinforcement learning techniques (e.g., Q-Learning, SARSA(lambda).  This module will not run a learning episode, 
it is meant only to store the MDP and staore and adapt the Q-Value function.  Updates and statistics
concerning the MDP can be accessed over an RPC port opened by this module.

\section intro_sec Description
\section lib_sec Libraries
YARP

\section parameters_sec Parameters
None.

\section portsa_sec Ports Accessed
None.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Linux.

\section example_sec Example Instantiation of the Module

\author Stephen Hart

Copyright (C) 2010 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/reinforcementLearning/main.cpp.
**/

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>

#include <iostream>
#include <string.h>
#include <stdlib.h>

#include <MarkovDecisionProcess.h>

using namespace yarp::os;
using namespace std;

#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_LOAD VOCAB4('l','o','a','d')
#define COMMAND_VOCAB_SAVE VOCAB4('s','a','v','e') //
#define COMMAND_VOCAB_RESET VOCAB4('r','e','s','e') //
#define COMMAND_VOCAB_UPDATE VOCAB4('u','p','d','a') //
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k') //
#define COMMAND_VOCAB_QUIT VOCAB4('q','u','i','t') //

#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_GET_MAX_ACTION VOCAB4('m','a','x','_') //
//#define COMMAND_VOCAB_GET_REWARD VOCAB4('r','e','w','a')
#define COMMAND_VOCAB_GET_ENTROPY VOCAB4('e','n','t','r') //
#define COMMAND_VOCAB_GET_TRANSITION VOCAB4('t','r','a','n') //
#define COMMAND_VOCAB_GET_NUM_STATES VOCAB4('s','t','a','t') //
#define COMMAND_VOCAB_GET_NUM_ACTIONS VOCAB4('a','c','t','i') //
#define COMMAND_VOCAB_GET_RULES VOCAB4('r','u','l','e')

#define COMMAND_VOCAB_SET VOCAB3('s','e','t') //
#define COMMAND_VOCAB_SET_STATE VOCAB4('s','t','a','t') //
#define COMMAND_VOCAB_SET_RULE VOCAB4('r','u','l','e')

#define COMMAND_VOCAB_ADD VOCAB3('a','d','d') //
#define COMMAND_VOCAB_ADD_GOAL VOCAB4('g','o','a','l') //

class ReinforcementLearningModule : public RFModule {

    Port handlerPort; // to handle messagees
    Semaphore mutex;
    
    MarkovDecisionProcess *mdp;
    
    // configuration params
    int num_states;
    int num_actions;
    string value_file;
    MarkovDecisionProcess::UpdateRules learning_rule;

    // runtime params
    int state;
    int action;
    double reward;

public:

    ReinforcementLearningModule() :
        value_file(""),
        num_states(0),
        num_actions(0),
        state(0),
        action(0),
        reward(0),
        learning_rule(MarkovDecisionProcess::Q_LEARNING)
    {
    }
    
    ~ReinforcementLearningModule() { }
  

    bool configure(ResourceFinder &rf)  {

        string iCubDir(getenv("ICUB_ROOT"));

        if(rf.check("num_states")) {
            num_states=rf.find("num_states").asInt();
        } 

        if(rf.check("num_actions")) {
            num_actions=rf.find("num_actions").asInt();
        } 

        if(rf.check("value_file")) {
            value_file=rf.find("value_file").asString().c_str();
        } 

        if(rf.check("learning_rule")) {
            string tmp=rf.find("learning_rule").asString().c_str();
            if(tmp=="Q_LEARNING") learning_rule = MarkovDecisionProcess::Q_LEARNING;
            if(tmp=="SARSA_LAMBDA") learning_rule = MarkovDecisionProcess::SARSA_LAMBDA;
        } 

        handlerPort.open("/reinforcementLearningModule/rpc:i");
        attach(handlerPort);
        attachTerminal();

        // create the MDP
        mdp = new MarkovDecisionProcess(num_states,num_actions,value_file);
        num_states = mdp->getNumStates();
        num_actions = mdp->getNumActions();

        return true;
    }

    bool respond(const Bottle &command, Bottle &reply)  {
     
        bool ok = false;
        bool rec = false; // is the command recognized?

        mutex.wait();

        switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP:
            rec = true;
            {
                cout << endl << "Reinforcement Learning Module" << endl;
                cout << "-----------------------------" << endl << endl;
                cout << "reset\t\t\t\t\t\tresets and clears all of the MDP information" << endl;
                cout << "get states\t\t\t\t\treturns the number of states in the MDP" << endl;
                cout << "get actions\t\t\t\t\treturns the number of actions in the MDP" << endl;
                cout << "get max_action [explore=0|1]\t\t\treturns the action for the current state with highest Q-value (uses exploration if flag is set)" << endl;
                cout << "get entropy [s] [a]\t\t\t\treturns the entropy of the resulting next state distribution when taking action 'a' in the state 's'" << endl;
                cout << "get transprob [s] [a] [sp]\t\t\treturns the probability of the transitioning from state 's' to state 'sp' after taking action 'a'" << endl;
                cout << "get update rules\t\t\t\tlists the available update rules" << endl;
                cout << "update [s] [a] [r] [sp]\t\t\t\tupdates the Q-Value function for taking action 'a' in state 's', receiving reward 'r' and transitioning to state 'sp'" << endl;
                cout << "add goal [s]\t\t\t\t\tadds state 's' to the goal set" << endl;
                cout << "set state [s]\t\t\t\t\tsets the current state to s" << endl;
                cout << "set rule [Q_LEARNING | SARSA_LAMBADA]\t\tsets the current update rule" << endl;
                cout << "save [filename]\t\t\t\t\tsaves the MDP statistics and value function to 'filename'"<<endl;
                cout << "load\t\t\t\t\t\tloads the value function from the default filename" << endl << endl;
                cout << "-----------------------------" << endl << endl;
                ok = true;
            }        
            break;
        case COMMAND_VOCAB_SET:
            switch (command.get(1).asVocab()) { 
            case COMMAND_VOCAB_SET_STATE:
                rec = true;
                {
                    int s = command.get(2).asInt();
                    cout << "num states : " << num_states << endl;
                    if( (s < 0) || (s >= num_states) ) {
                        cout << "RL Module -- Invalid state: " << s << endl; 
                        ok = false;
                        break;
                    } else  {
                        state = s;
                        cout << "RL Module -- setting state: " << state << endl;
                        mdp->setState(state);
                    }
                }
                break;
            case COMMAND_VOCAB_SET_RULE:
                rec = true;
                {
                    ConstString rule = command.get(2).asString();
                    if(rule == "Q_LEARNING") {
                        learning_rule = MarkovDecisionProcess::Q_LEARNING;
                        cout << "rule: " << rule.c_str() << endl;
                    } else if(rule == "SARSA_LAMBDA") {
                        learning_rule = MarkovDecisionProcess::SARSA_LAMBDA;
                        cout << "rule: " << rule.c_str() << endl;
                    } else {
                        cout << "unknown update rule..." << endl;
                    }
                    
                }
                break;
            }
            ok = true;
            break;        
        case COMMAND_VOCAB_GET:
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_GET_NUM_STATES:
                rec = true;
                {
                    reply.addInt(num_states);
                }
                break;
            case COMMAND_VOCAB_GET_NUM_ACTIONS:
                rec = true;
                {
                    reply.addInt(num_actions);
                }
                break;
            case COMMAND_VOCAB_GET_RULES:
                rec = true;
                {
                    cout << "Q_LEARNING" << endl;
                    cout << "SARSA_LAMBDA" << endl;
                }
                break;
            case COMMAND_VOCAB_GET_MAX_ACTION:
                rec = true;
                {
                    if(command.size() == 3) {
                        int a;
                        bool exp = command.get(2).asInt();
                        a = mdp->getMaxAction(state,(bool)exp);
                        reply.addInt(a);
                        cout << "got max action for state["<<state<<"]: " << a << ", exploration=" << exp << endl;
                    } else {
                        cout << "Incorrect get max action format, usage: get max_action [explore=0|1]" << endl;
                    }
                }
                break;
            case COMMAND_VOCAB_GET_ENTROPY:
                rec = true;
                {
                    if(command.size() == 4) {
                        rec = true;
                        int s = command.get(2).asInt();
                        int a = command.get(3).asInt();
                        double entropy = mdp->getEntropy(s,a);
                        cout << "got entropy for state["<<s<<"], action["<<a<<"] --> entropy="<<entropy<<endl;
                        reply.addDouble(entropy);
                    } else {
                        cout << "Incorrect update format, usage: get entropy [state] [action]" << endl;
                    }
                }
                break;
            case COMMAND_VOCAB_GET_TRANSITION:
                rec = true;
                {
                    if(command.size() == 5) {
                        rec = true;
                        int s = command.get(2).asInt();
                        int a = command.get(3).asInt();
                        int sp = command.get(4).asInt();
                        double pr = mdp->getTransitionProb(s,a,sp);
                        cout << "Probability=" << pr << ", for transitioning from state["<<s<<"] to state[" << sp << "], for action["<<a<<"]"<<endl;
                        reply.addDouble(pr);
                    } else {
                        cout << "Incorrect update format, usage: get transprob [state] [action] [next_state]" << endl;
                    }
                }
                break;
            }
            ok = true;
            break;
        case COMMAND_VOCAB_ADD:
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_ADD_GOAL:
                rec = true;
                {
                    int g = command.get(2).asInt();
                    cout << "trying to add goal state : " << g << endl;
                    if( (g < 0) || (g >= num_states) ) {
                        cout << "RL Module -- Invalid goal state: " << g << endl; 
                        ok = false;
                        break;
                    } else  {
                        mdp->addGoalState(g);
                        cout << "RL Module -- setting state: " << state << endl;
                    }
                }
            }
            ok = true;
            break;
        case COMMAND_VOCAB_SAVE:
            if(command.size() == 2) {
                rec = true;
                ConstString fName = command.get(1).asString();
                mdp->setValueFile(fName.c_str());
                mdp->writeValueFunction();
                ok = true;
            }
            break;    
        case COMMAND_VOCAB_LOAD:
            rec = true;
            mdp->loadValueFunction();
            ok = true;            
            break;    
        case COMMAND_VOCAB_RESET:
            rec = true;
            mdp->reset();
            ok = true;            
            break;    
        case COMMAND_VOCAB_UPDATE:
            if(command.size() == 5) {
                rec = true;
                int s = command.get(1).asInt();
                int a = command.get(2).asInt();
                double r = command.get(3).asDouble();
                int sp = command.get(4).asInt();
                cout << "RL module updating MDP with: s["<<s<<"]-- a["<<a<<"] --> s["<<sp<<"], reward="<<r<<endl;
                mdp->updateRule(s,a,r,sp,learning_rule);
                ok = true;
            } else {
                cout << "Incorrect update format, usage: update [state] [action] [reward] [next_state]" << endl;
            }
            break;
        }
        
        mutex.post();
        
        if (!rec)
            ok = RFModule::respond(command,reply);
        
        if (!ok) {
            reply.clear();
            reply.addVocab(COMMAND_VOCAB_FAILED);
        }
        else
            reply.addVocab(COMMAND_VOCAB_OK);

        return ok;
    }
    

    bool close() {
        cout << "Closing RL Module..." << endl; 
        handlerPort.close();
        return true;
    }
  
    double getPeriod() {
        return 1;
    }
    
    bool updateModule() {
        Time::delay(1);
        return true;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    ReinforcementLearningModule mod;

    ResourceFinder rf;
    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);

    cout<<"Starting RL Module..."<<endl;    
    return mod.runModule(rf);

}

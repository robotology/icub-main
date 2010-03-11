// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _MDP_H__
#define _MDP_H__

#include <string>
#include <vector>
#include <map>

class MarkovDecisionProcess {
    
private:
    
    static const double SMALL_VAL=0.000001;
    static const int MAX_GOAL_STATES=40;
    
    int _numStates;
    int _numActions;
    
    double _initValue;
    
    std::map<int, double>_values;
    //  map<int, int>_policy;
    std::map<int, double>_entropy;
    std::map<int, double>_eligibility;
    std::map<int, double>_transitions;
    std::map<int, int>_numTransitions;
    std::vector<double>_rewards;
    std::vector<double>_maxVals;
    
    int _state;
    int _action;
    int _last_state;
    int _last_action;

    std::vector<int> _goalStates; 
    std::vector<int> _policy;
    std::string _valueFile;
    bool _valueFileSet;


  public:
    
    // e-greedy rate
    static const double EPSILON = 0.2;
    
    // update rule parameters
    static const double ALPHA = 0.1;
    static const double LAMBDA = 0.8;
    static const double GAMMA = 0.8;
    
    // learning rules
    enum UpdateRules {
      Q_LEARNING=0,
      SARSA_LAMBDA
    };
    
    MarkovDecisionProcess(int numStates=0, int numActions=0, std::string valueFile="");
    ~MarkovDecisionProcess();
    
    int getNumStates() { return _numStates; }
    int getNumActions() { return _numActions; }
    
    void setState(int s);
    void setValueFile(std::string valueFile) {
      _valueFile = valueFile;
      _valueFileSet = true;
    }

    double getValue(int s, int a);
    void setValue(int s, int a, double v);  
  
    double getEntropy(int s, int a);  
    void setEntropy(int s, int a, double v);

    double getTransitionProb(int s, int a, int sp);
    void setTransitionProb(int s, int a, int sp, double v);

    int getTransitions(int s, int a, int sp);
    void setTransitions(int s, int a, int sp, int n);

    double getEligibility(int s, int a);
    void setEligibility(int s, int a, double v);

    void addGoalState(int s);
    int isGoalState(int s);

    int loadValueFunction();
    void writeValueFunction(double reward=0.0);

    void updateRule(int s, int a, double r, int sp, UpdateRules rule);
    void updateTransitionProbabilities(int s, int a, int sp);
    int getMaxAction(int s, int explore);
    double getMaxReward(int &s, int &a);

    void reset();
};

#endif

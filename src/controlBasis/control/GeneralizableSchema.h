// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
 * This class implementes a generalizable Control Basis Schema whose state is 
 * evaluted as the dynamic state of the available actions (both controllers
 * and schema used hierarchically).  Reward is accomplished though the convergence of 
 * controllers that engage stimuli from the environment as reference signals.
 * The dynamic state of actions is evaluated as s={UNKNOWN,UNDEFINED,UNCONVERGED,CONVERGED}.
 *
 * This schema is "generalizable" because it parameterizes the control actions
 * by resources it has learned are most likely to lead to success (reward)
 * based on the run-time context.  For example, a robot can accomplish a 
 * "reaching" action with its left or right arm depending on the location
 * of the object to be reached to.  
 *
 * !!!BEWARE!!!! This is work in progress (and not all the pieces are implemented yet...)
 *
 * Author: Stephen Hart
 **/
#ifndef _GENERALIZABLE_SCHEMA__H_
#define _GENERALIZABLE_SCHEMA__H_

#include "Schema.h"

namespace CB {

  class GeneralizableSchema : public Schema {

  public:
    
    /**
     * Constructor
     **/
    GeneralizableSchema(std::vector<ControllerParameters> controllerParams, std::vector<std::string> subSchema, int limit);

    /**
     * Destructor
     **/
    ~GeneralizableSchema();

    virtual double evaluateReward(int state, int last_state, int action, int last_action);
    virtual int evaluateState();
    virtual std::string getStringFromState(int s);
    virtual int getStateFromString(std::string str);     

  };

}


#endif

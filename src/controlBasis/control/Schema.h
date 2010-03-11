// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _SCHEMA__H_
#define _SCHEMA__H_

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Port.h>
#include <vector>
#include <math.h>
#include <MarkovDecisionProcess.h>

#include "ControlBasisAction.h"
#include "ControllerParameters.h"
#include "RunnableControlLaw.h"

namespace CB {

    /**
     * This is a (vritual and abstract) class that implements a ControlBasis Schema
     * as a Markov Decision Process (or MDP).  In such a way, the policy for the schema
     * can be learned using reinforcement learning, if the children of this class implement
     * the pure virtual functions that evaluate the state and the reward based on
     * whatever necessary criteria or context.  
     *
     * The basic idea of a schema is to evaluate the context (or state), and then choose an action 
     * that statistically maximized the amount of expected future return (or reward).  Each state 
     * is considered to be a discrete string of predicates, each of which can be evaluated as a 
     * discrete value.  For example, if the state is conisdered to be a number or controller 
     * activations, the state string  would be a binary word the length of the number of controllers. 
     *
     * Each schema is allowed to use ControlBasis Controllers or other Schema in its action set.
     * The necessary information for these actions are passed in to the Schema constructor.  
     * Multi-objective control laws of size "limit" can be included in the action set if desired.
     *  
     * When used hierarchically, each schema will report its status in terms of a single 4-valued
     * state predicate (much like the primitive Controllers), where 
     * s={UNKNOWN,UNDEFINED,UNCONVERGED,CONVERGED}.  If the schema is not running it will be 
     * considereed UNKNOWN.  If it is not applicable in the current context, it will be considered
     * UNDEFINED.  If it is running, but has not reached a goal state, then it will be considered
     * UNCONVERGED.  In goal states, or states where the probability of transitioning to a state
     * with higher value is sufficiently low, the state will be considered CONVERGED.
     *
     **/
    class Schema : public ControlBasisAction {
        
    protected:

        /**
         * a name for the schema
         **/
        std::string schemaName;

        /** 
         * The number of actions (w.r.t. the MDP). Will be equal to the number of
         * multii-objective control laws (including laws of size 1) plus the number
         * of schema that are to be used hierarchically.
         **/
        int numActions;

        /*
         * The number of states (w.r.t., the MDP). Will be number of predicate values
         * raised to the number of predicates.
         **/
        int numStates;

        /**
         * an interger state descriptor (in constrast to the string value, although
         * there will exist a one-to-one mapping.
         **/
        int internalState;

        /**
         * the last state the Schema was in before it transitioned to the current state.
         **/
        int internalStateLast;

        /**
         * the number of state predicates to be evaluated in the state string.
         **/
        int numStatePredicates;

        /**
         * The number of values each predicate can assume.  For binary predicates, this will be 2.
         **/
        int numPredicateValues;

        /**
         * the state string
         **/
        std::string internalStateString;

        /**
         * the current action running.
         **/
        int action;

        /**
         * the last action that was taken.
         **/
        int actionLast;

        /**
         * the total amount of cumulative reward since the schema began running.
         **/
        double cumulativeReward;

        /**
         * a list of the primitive controller information for the schema.
         * This should be passed to the obejct in the constructur.
         **/  
        std::vector<ControllerParameters> controllerParameters;

        /**
         * a list of the names of schema that can be used hierarchically as 
         * temporally extended actions in the current (top level) schema.
         **/
        std::vector<std::string> schemaList;

        /**
         * the controllers
         **/
        std::vector<Controller *> controllers;

        /**
         * the schema
         **/
        std::vector<Schema *> schema;

        /**
         * each controller will be run as a single- or multi-objective control law.
         * this is that control law.
         **/
        RunnableControlLaw controlLaw;

        /**
         * a MDP helper object that stores the value function and handles action selection.
         **/
        MarkovDecisionProcess *mdp;

        /**
         * an action mask that will turn integer action id's into strings specifying 
         * inclusion and order in the multi-objective control law set.
         **/
        std::vector<std::string> actionMask;

        /**
         * exploration flag.  If set to true, e-greedy exploration will occur.
         * If set to false, the MDP will always choose the "best" action
         * from the current state.
         **/
        bool explore;

        /**
         * the limit of number of controllers that could be included in the multi-objective laws.
         **/
        int compositeLimit;

        /**
         * the number of primitive controllers
         **/
        int numPrimitiveControllers;

        /** 
         * the number of composite controllers (not including laws of size 1).
         **/
        int numCompositeControllers;

        /**
         * the number of sub-schema
         **/
        int numSchema;

        /**
         * the integer IDs of the subschema in the action set.
         **/
        int schema_id;

        /**
         * the value of the current state 
         **/
        double value;

        /**
         * the value of the last state
         **/
        double valueLast;
    
        /**
         * the estimated change in value w.r.t. the number of iterations
         **/
        double valueDot;
        
        /**
         * storage for the value as the schema runs
         */
        std::vector<double> valueStore;

        /**
         * storage for the estimated value derivative as the schema runs
         */
        std::vector<double> valueDotStore;       
        

    public:
        
        /** 
         * constructor 
         **/
        Schema(std::vector<ControllerParameters> controllerParams, std::vector<std::string> subSchema, int limit, int nStatePredicates, int nPredicateValues);

        /** 
         * destructor
         **/
        ~Schema() {
            std::cout << "Schema destructor..." << std::endl;
            resetSchema();

            for(int i=0; i<controllers.size(); i++) {
                delete controllers[i];
            }
            controllers.clear();                
        }       
        
        /**
         * Inherited update fuction from ControlBasisAction class
         **/
        virtual bool updateAction();

        /**
         * Inherited start fuction from ControlBasisAction class
         **/
        virtual void startAction();

        /**
         * Inherited stop fuction from ControlBasisAction class
         **/
        virtual void stopAction();

        /**
         * Inherited postData fuction from ControlBasisAction class
         **/
        virtual void postData();  

        /**
         * reset funtion
         **/
        void resetSchema();  

        /**
         * returns current schema value
         */
        double getSchemaValue() {
            return value;
        }

        /**
         * sets the name of the schema
         **/
        void setName(std::string name) { 
            schemaName=name; 
            if(mdp==NULL) return;

            std::string iCubDir(getenv("ICUB_ROOT"));
            std::string vfile = iCubDir + "/app/controlBasis/conf/" + schemaName + ".xml";
            mdp->setValueFile(vfile);
            std::cout << "setting value function name: " << vfile << std::endl; 
        }

        /**
         * returns current controller potential
         */
        double getSchemaValueDot() {
            return valueDot;
        }

        /**
         * gets the string value of the current state
         **/
        std::string getInternalStateString() { 
            internalStateString = getStringFromState(internalState);
            return internalStateString;
        }

        /**
         * adds goals to the MDP
         **/
        bool addGoal(std::string g) {
            if(mdp==NULL) return false;
            mdp->addGoalState(getStateFromString(g));
        }

        /**
         * saves the MDP information to an .xml configuration file
         **/
        void save() { 
            if(mdp==NULL) return;
            mdp->writeValueFunction(cumulativeReward); 
        }

        /**
         * loads the MDP information from the .xml configuration file
         **/
        void load() { 
            if(mdp==NULL) return;
            mdp->loadValueFunction(); 
        }

        // new abstract functions
        /**
         * evalues the reward based on the state and action information
         **/
        virtual double evaluateReward(int state, int last_state, int action, int last_action)=0;

        /**
         * evaluates the current state value based on whatever context is necessary
         **/
        virtual int evaluateState()=0;

        /**
         * gets the state string from the ID
         **/
        virtual std::string getStringFromState(int s)=0;

        /**
         * gets the interger state ID from the state string
         **/
        virtual int getStateFromString(std::string str)=0;


  };
 

}

#endif

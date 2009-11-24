// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Assif Mirza
 * email:   assif.mirza@robotcub.org
 * website: www.robotcub.org
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

#ifndef __ACTION_SELECT_H__
#define __ACTION_SELECT_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <map>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/Actions.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace iha {
        class ActionSelect;
    }
}

using namespace iCub::iha;

/**
@ingroup icub_iha_ActionSelection

\brief Action Selection Process class 

For description of the algorithm see \link icub_iha_ActionSelection Action Selection (IHA) \endlink

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

 */
class iCub::iha::ActionSelect {

public:
    ActionSelect() {
    }
    ~ActionSelect() {
    }

    /**
     * Read the data from the bottles for each horizon 
     * and populate the probability and action frequency arrays
     * \param neighbourlist[] Neighbour list bottles
     */
    bool parseNeighbourList(Bottle* neighbourlist[]);

    /** 
     * Select an action given the current experience and action probabilites
     */
    int selectAction();
    
    /**
     * Initialize
     */
    void init(Searchable &config);

    /** 
     * read actions from sequence files
     * \param filename sequence file name
     * \param seqdir directory parh to file
     */
    void getActions(ConstString filename);

    /**
     * Set the next behaviour set based on the passed action
     * \param act action
     */
    void updateNextBehaviourSet(int act);

    /**
     * stochastically select a number from a weighted distrubution (roulette wheel)
     *
     * e.g. if distribution is  0.1   0.5   0.3   0.1
     *
     * then there should be a 10% chance of picking choices 0 or 3
     * a 30% chance of picking 2 and a 50% chance of picking choice number 2
     *
     * \param distribution vector of doubles that is the prob distribution
     */
    int chooseFromDistribution(vector<double> distribution);

    /**
     * Pick a random action
     */
    int chooseRandom() 
    {
        return rand() % NUM_ACTIONS;
    }

    /**
     * Pick a random number (action) from a set (vector actually) of numbers
     * \param aset vector of actions
     */
    int chooseRandomFromSet(vector<int> aset) 
    {
        int r = rand() % aset.size();
        return aset[r];
    }

    /**
     * Pick a random number (action) from the next behaviour set
     * \param next_behaviour_set vector of actions
     */
    int selectRandomAction(vector<int> next_behaviour_set) ;

    /**
     * Get the next behaviour set for the given action
     * \param act action
     */
    vector<int> getNextBehaviourSet(int act) {
        return iCubActions.getActionNextBehaviourSet(act);
    }


private:
    Actions iCubActions;

    int NUM_ACTIONS;  // needs to be total nmber of actions (not just for default behaviour set)
    vector<std::string> action_commands;
    vector<int> next_behaviour_set;

    int numHorizons;
    int horizons[];

    bool use_behaviour_sets;
    double neighbour_radius;

    double Temperature;
    double temp_dec;
    
    bool select_chosen; 
    bool select_random;

    int wheelResolution;

    /**
     * elist - list of experiences that we will use to decide next action
     *         sorted shortest distance first
     *   format: (distance, (horizon, expid))
     */
    multimap<double,pair<int,int> > elist;

    /**
     * values - map of values for each experience
     *   format: ((horizon,expid),value)
     */
    map< pair<int,int>, double> values;

    /**
     * actfreq_distributions - Action frequency distribution
     *   format: ((horizon,expid), prob_of_act0, prob_of_act1, ... )
     */
    map< pair<int,int>, vector < double > > actfreq_distributions;

    /**
     * Mass - weighting of experiences according to
     *        how many experiences were merged with this one
     *        (we infer this from the action frequency distribution)
     *   formatL ((horizon,expid), mass, ... )
     */
    map< pair<int,int>, double > masses;

    /**
     * Distances - for debug
     */
    map< pair<int,int>, double > distances;


};

#endif
